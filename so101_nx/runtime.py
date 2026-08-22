"""统一入口使用的双状态运行时。"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor

from .calibration import load_calibration
from .constants import COMMAND_TOPIC
from .control import CloudForwardControl, ControlBackend, RosTopicControl
from .feetech import FeetechArm
from .keyboard import KeyboardReader
from .teaching import TeachingSession

LOGGER = logging.getLogger("so101_nx.runtime")

STATE_CONTROL = "control"
STATE_TEACHING = "teaching"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--leader-port", required=True, help="主臂串口，例如 /dev/ttyACM0")
    parser.add_argument("--follower-port", required=True, help="从臂串口，例如 /dev/ttyACM1")
    parser.add_argument("--leader-calibration", type=Path, required=True, help="主臂 LeRobot 标定 JSON")
    parser.add_argument("--follower-calibration", type=Path, required=True, help="从臂 LeRobot 标定 JSON")
    parser.add_argument("--output-root", type=Path, default=Path("/data/so101_raw"), help="示教原始数据目录")
    parser.add_argument("--task", default="", help="写入每个 episode 元数据的任务描述")
    parser.add_argument("--fps", type=float, default=30.0, help="控制和采集频率")
    parser.add_argument(
        "--control-backend",
        choices=("ros2", "cloud"),
        default="ros2",
        help="启动时的从臂控制后端，默认 ros2；cloud 仅预留结构",
    )
    parser.add_argument("--command-topic", default=COMMAND_TOPIC, help="ROS2 控制命令话题")
    parser.add_argument("--command-timeout-s", type=float, default=0.5, help="控制命令超时秒数")
    parser.add_argument("--max-relative-target-deg", type=float, default=15.0, help="单周期最大目标变化角度")
    parser.add_argument("--color-topic", default="/camera/camera/color/image_raw", help="RealSense 彩色图像话题")
    parser.add_argument(
        "--depth-topic",
        default="/camera/camera/aligned_depth_to_color/image_raw",
        help="RealSense 对齐深度图像话题",
    )
    parser.add_argument("--no-depth", action="store_true", help="示教时不记录深度图像")
    parser.add_argument("--jpeg-quality", type=int, default=92, help="彩色 JPEG 质量")
    parser.add_argument("--max-image-age-ms", type=float, default=250.0, help="允许图像帧的最大年龄")
    parser.add_argument("--skip-start-prompt", action="store_true", help="跳过启动前的安全确认")
    parser.add_argument("--log-level", default="INFO", choices=("DEBUG", "INFO", "WARNING", "ERROR"))
    args = parser.parse_args()
    if args.fps <= 0:
        parser.error("--fps 必须为正数")
    if args.command_timeout_s <= 0:
        parser.error("--command-timeout-s 必须为正数")
    if not 1 <= args.jpeg_quality <= 100:
        parser.error("--jpeg-quality 必须位于 [1, 100]")
    if args.max_image_age_ms <= 0:
        parser.error("--max-image-age-ms 必须为正数")
    return args


class SO101Runtime:
    """只在从臂控制和主从示教两个状态间切换的状态机。"""

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.executor = SingleThreadedExecutor()
        self.leader = FeetechArm(args.leader_port, load_calibration(args.leader_calibration))
        self.follower = FeetechArm(args.follower_port, load_calibration(args.follower_calibration))
        self.state = ""
        self.control = None  # type: Optional[ControlBackend]
        self.teaching = None  # type: Optional[TeachingSession]
        self.running = True

    def connect_hardware(self) -> None:
        self.leader.connect()
        self.follower.connect()
        self.leader.verify_position_limits()
        self.follower.verify_position_limits()
        self.leader.disable_torque()
        self.follower.disable_torque()

    def _leave_control(self) -> None:
        if self.control is None:
            return
        try:
            self.control.stop()
        finally:
            if isinstance(self.control, RosTopicControl):
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
        if self.args.control_backend == "ros2":
            backend = RosTopicControl(
                self.follower,
                self.leader,
                self.args.fps,
                self.args.command_timeout_s,
                self.args.max_relative_target_deg,
                self.args.command_topic,
            )
            self.executor.add_node(backend)
        else:
            backend = CloudForwardControl(self.follower)
        self.control = backend
        self.control.start()
        self.state = STATE_CONTROL
        LOGGER.info("当前状态：从臂控制。按 r 切换到主从示教，按 q 退出。")

    def enter_teaching(self) -> None:
        if self.state == STATE_TEACHING:
            return
        self._leave_control()
        self.teaching = TeachingSession(
            self.executor,
            self.leader,
            self.follower,
            self.args.output_root,
            self.args.task,
            self.args.fps,
            self.args.color_topic,
            None if self.args.no_depth else self.args.depth_topic,
            self.args.leader_calibration,
            self.args.follower_calibration,
            self.args.jpeg_quality,
            self.args.max_image_age_ms,
            self.args.max_relative_target_deg,
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
        if not self.args.skip_start_prompt:
            input("已连接主从臂且扭矩失能。确认工作空间安全后按回车启动。")
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
        try:
            self.leader.disable_torque()
        except Exception:
            pass
        self.leader.close()
        self.follower.close()
        self.executor.shutdown()


def main() -> int:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level), format="%(asctime)s %(levelname)s %(message)s")
    rclpy.init(args=None)
    runtime = SO101Runtime(args)
    try:
        runtime.run()
    except KeyboardInterrupt:
        LOGGER.info("收到 Ctrl+C，正在安全退出。")
    except Exception as exc:
        LOGGER.exception("运行时失败：%s", exc)
        return 1
    finally:
        runtime.shutdown()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
