"""双状态运行时：在从臂控制与主从示教之间切换，并管理 rclpy 生命周期。"""

from __future__ import annotations

import logging
import sys
import time
from typing import Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor

from config import Config

from hardware.keyboard import KeyboardReader
from hardware.motors.calibration import load_calibration
from hardware.motors.feetech import FeetechArm

from .control.base import ControlBackend
from .control.cloud_backend import CloudForwardControl
from .control.ros2_backend import RosTopicControl
from .teaching import TeachingSession

LOGGER = logging.getLogger("state.runtime")

STATE_CONTROL = "control"
STATE_TEACHING = "teaching"


class SO101Runtime:
    """只在从臂控制和主从示教两个状态间切换的状态机。"""

    def __init__(self, config: Config) -> None:
        self.config = config
        self.executor = SingleThreadedExecutor()
        # 仅从臂模式：leader_port 为 None 时 leader 为 None
        self.leader = (
            FeetechArm(config.leader_port, load_calibration(config.leader_calibration))
            if config.leader_port
            else None
        )
        self.follower = FeetechArm(config.follower_port, load_calibration(config.follower_calibration))
        self.state = ""
        self.control = None  # type: Optional[ControlBackend]
        self.teaching = None  # type: Optional[TeachingSession]
        self.running = True

    def connect_hardware(self) -> None:
        if self.leader is not None:
            self.leader.connect()
            self.leader.verify_position_limits()
            self.leader.disable_torque()

        self.follower.connect()
        self.follower.verify_position_limits()
        self.follower.disable_torque()

    def _leave_control(self) -> None:
        if self.control is None:
            return
        try:
            self.control.stop()
        finally:
            self.executor.remove_node(self.control)
            self.control.destroy_node()
            self.control = None

    def _leave_teaching(self) -> None:
        if self.teaching is None:
            return
        try:
            self.teaching.stop()
        finally:
            self.teaching = None

    def enter_control(self) -> None:
        if self.state == STATE_CONTROL:
            return
        self._leave_teaching()
        if self.config.control_backend == "ros2":
            backend = RosTopicControl(self.follower, self.leader, self.config)
        else:
            backend = CloudForwardControl(self.follower, self.leader, self.config)
        self.executor.add_node(backend)
        self.control = backend
        self.control.start()
        self.state = STATE_CONTROL
        LOGGER.info("当前状态：从臂控制。按 r 切换到主从示教，按 q 退出。")

    def enter_teaching(self) -> None:
        if self.state == STATE_TEACHING:
            return
        if self.leader is None:
            LOGGER.warning("缺少主臂参数，仅从臂控制")
            return
        self._leave_control()
        self.teaching = TeachingSession(
            self.executor,
            self.leader,
            self.follower,
            self.config.output_root,
            self.config.task,
            self.config.fps,
            self.config.color_topic,
            self.config.depth_topic if self.config.record_depth else None,
            self.config.leader_calibration,
            self.config.follower_calibration,
            self.config.jpeg_quality,
            self.config.max_image_age_ms,
            self.config.max_relative_target_deg,
        )
        episode_dir = self.teaching.start()
        self.state = STATE_TEACHING
        LOGGER.info("当前状态：主从示教。正在记录 %s；按 c 返回从臂控制，按 q 退出。", episode_dir)

    def handle_key(self, key: str) -> None:
        if key == "r":
            self.enter_teaching()
        elif key == "c":
            self.enter_control()
        elif key == "q":
            self.running = False
        elif key and key not in ("\n", "\r"):
            LOGGER.warning("未知按键 %r；可用按键为 r、c、q。", key)

    def run(self) -> None:
        self.connect_hardware()
        if not self.config.skip_start_prompt:
            prompt = "已连接从臂" + ("和主臂" if self.leader is not None else "") + "且扭矩失能。确认工作空间安全后按回车启动。"
            input(prompt)
        self.enter_control()
        with KeyboardReader() as keyboard:
            while self.running:
                self.executor.spin_once(timeout_sec=0.005)
                key = keyboard.read_key()
                if key is not None:
                    self.handle_key(key)
                if self.state == STATE_CONTROL and self.control is not None:
                    self.control.step()
                elif self.state == STATE_TEACHING and self.teaching is not None:
                    self.teaching.step()
                time.sleep(0.001)

    def shutdown(self) -> None:
        self._leave_control()
        self._leave_teaching()
        try:
            self.follower.disable_torque()
        except Exception:
            pass
        if self.leader is not None:
            try:
                self.leader.disable_torque()
            except Exception:
                pass
            self.leader.close()
        self.follower.close()
        self.executor.shutdown()


def main() -> int:
    config = Config()
    logging.basicConfig(level=config.log_level, format="%(asctime)s %(levelname)s %(message)s")
    rclpy.init(args=None)
    runtime = SO101Runtime(config)
    try:
        runtime.run()
    except KeyboardInterrupt:
        LOGGER.info("收到 Ctrl+C，正在退出。")
    except Exception as exc:
        LOGGER.exception("运行时失败：%s", exc)
        return 1
    finally:
        runtime.shutdown()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
