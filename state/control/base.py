"""从臂控制后端的公共基类：话题发布、控制节拍与安全执行路径。"""

from __future__ import annotations

import json
import logging
import time
from typing import Dict, Optional

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from config import Config
from hardware.motors.calibration import degrees_to_raw
from hardware.motors.feetech import FeetechArm
from hardware.motors.joints import JOINT_NAMES
from hardware.motors.safety import clip_goals

LOGGER = logging.getLogger("state.control")


class ControlBackend(Node):
    """控制后端公共部分。

    本类负责所有后端共同的工作：
    - 发布 follower/leader 状态、action、status 四个话题；
    - control_fps 节拍控制与落后追赶；
    - 目标限幅、角度到编码器转换、扭矩生命周期与异常兜底。

    子类只需实现 `_next_target_degrees` 提供目标来源（单条命令、动作
    chunk 或云端转发），不直接操作电机。
    """

    NODE_NAME = "so101_control_backend"

    # 无可用目标时发布的状态名，子类按命令源语义覆写。
    idle_status_state = "waiting_for_command"

    def __init__(self, follower: FeetechArm, leader: FeetechArm, config: Config) -> None:
        Node.__init__(self, self.NODE_NAME)
        self.follower = follower
        self.leader = leader
        self.config = config
        self.period_ns = int(1e9 / config.control_fps)
        self.max_relative_target_deg = config.max_relative_target_deg
        self.next_tick_ns = 0
        self.torque_enabled = False
        self.last_status_state = ""
        self.follower_state_pub = self.create_publisher(Float64MultiArray, config.follower_state_topic, 10)
        self.leader_state_pub = self.create_publisher(Float64MultiArray, config.leader_state_topic, 10)
        self.action_pub = self.create_publisher(Float64MultiArray, config.action_topic, 10)
        self.status_pub = self.create_publisher(String, config.status_topic, 10)

    def _next_target_degrees(self, follower_degrees: Dict[str, float], now_ns: int) -> Optional[Dict[str, float]]:
        """返回本周期要执行的目标角度，无可用目标时返回 None。

        返回 None 时基类失能扭矩并发布 idle_status_state。
        """
        raise NotImplementedError

    def _send_action_degrees(self, target: Dict[str, float], follower_degrees: Dict[str, float]) -> Dict[str, float]:
        """限幅、转换并下发目标，返回实际执行的动作角度。"""
        action_degrees = clip_goals(target, follower_degrees, self.max_relative_target_deg)
        action_raw = {
            joint: degrees_to_raw(action_degrees[joint], self.follower.calibration[joint])
            for joint in JOINT_NAMES
        }
        if not self.torque_enabled:
            self.follower.enable_torque()
            self.torque_enabled = True
        self.follower.write_goal_positions_raw(action_raw)
        self._publish_positions(self.action_pub, action_degrees)
        return action_degrees

    def _publish_status(self, state: str, **extra: object) -> None:
        if state == self.last_status_state and state not in ("hardware_error", "invalid_command", "invalid_chunk"):
            return
        payload = {
            "state": state,
            "joint_names": list(JOINT_NAMES),
            "position_unit": "degrees",
            "timestamp_ns": time.monotonic_ns(),
        }
        payload.update(extra)
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"))
        self.status_pub.publish(message)
        self.last_status_state = state

    @staticmethod
    def _publish_positions(publisher, positions: Dict[str, float]) -> None:
        message = Float64MultiArray()
        message.data = [float(positions[joint]) for joint in JOINT_NAMES]
        publisher.publish(message)

    def start(self) -> None:
        self.follower.configure_follower()
        self.next_tick_ns = time.monotonic_ns()

    def step(self) -> None:
        now_ns = time.monotonic_ns()
        if now_ns < self.next_tick_ns:
            return
        self.next_tick_ns += self.period_ns
        if now_ns - self.next_tick_ns > self.period_ns:
            self.next_tick_ns = now_ns + self.period_ns
        try:
            _, follower_degrees = self.follower.read_positions_degrees()
            self._publish_positions(self.follower_state_pub, follower_degrees)
            if self.leader is not None:
                _, leader_degrees = self.leader.read_positions_degrees()
                self._publish_positions(self.leader_state_pub, leader_degrees)
            target = self._next_target_degrees(follower_degrees, now_ns)
            if target is None:
                if self.torque_enabled:
                    self.follower.disable_torque()
                    self.torque_enabled = False
                self._publish_status(self.idle_status_state)
                return
            self._send_action_degrees(target, follower_degrees)
            self._publish_status("executing")
        except Exception as exc:
            self.get_logger().error("控制器错误：%s", exc)
            self.follower.disable_torque()
            self.torque_enabled = False
            self._publish_status("hardware_error", detail=str(exc))

    def stop(self) -> None:
        self.follower.disable_torque()
        self.torque_enabled = False
        self._publish_status("stopped")
