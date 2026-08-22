"""默认控制后端：订阅 ROS2 关节命令并发布状态。"""

from __future__ import annotations

import json
import logging
import math
import time
from typing import Dict, List, Optional

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from config import ACTION_TOPIC, COMMAND_TOPIC, FOLLOWER_STATE_TOPIC, LEADER_STATE_TOPIC, STATUS_TOPIC
from hardware.motors.calibration import degrees_to_raw
from hardware.motors.feetech import FeetechArm
from hardware.motors.joints import JOINT_NAMES
from hardware.motors.safety import clip_goals

from .base import ControlBackend

LOGGER = logging.getLogger("state.control")


class RosTopicControl(Node, ControlBackend):
    """默认控制后端：订阅 ROS2 关节命令并发布状态。"""

    def __init__(
        self,
        follower: FeetechArm,
        leader: FeetechArm,
        control_fps: float,
        command_timeout_s: float,
        max_relative_target_deg: float,
        command_topic: str = COMMAND_TOPIC,
    ) -> None:
        Node.__init__(self, "so101_topic_controller")
        self.follower = follower
        self.leader = leader
        self.period_ns = int(1e9 / control_fps)
        self.command_timeout_ns = int(command_timeout_s * 1e9)
        self.max_relative_target_deg = max_relative_target_deg
        self.last_command = None  # type: Optional[List[float]]
        self.last_command_ns = 0
        self.next_tick_ns = 0
        self.torque_enabled = False
        self.last_status_state = ""
        self.create_subscription(Float64MultiArray, command_topic, self._on_command, 10)
        self.follower_state_pub = self.create_publisher(Float64MultiArray, FOLLOWER_STATE_TOPIC, 10)
        self.leader_state_pub = self.create_publisher(Float64MultiArray, LEADER_STATE_TOPIC, 10)
        self.action_pub = self.create_publisher(Float64MultiArray, ACTION_TOPIC, 10)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.command_topic = command_topic

    def _publish_status(self, state: str, **extra: object) -> None:
        if state == self.last_status_state and state not in ("hardware_error", "invalid_command"):
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

    def _on_command(self, message: Float64MultiArray) -> None:
        if len(message.data) != len(JOINT_NAMES):
            self.get_logger().error(
                "忽略包含 %d 个值的命令；应按顺序 %s 提供 %d 个值",
                len(message.data),
                list(JOINT_NAMES),
                len(JOINT_NAMES),
            )
            self._publish_status("invalid_command", received_values=len(message.data))
            return
        values = [float(value) for value in message.data]
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error("忽略包含 NaN 或 Inf 的命令")
            self._publish_status("invalid_command", reason="non_finite")
            return
        self.last_command = values
        self.last_command_ns = time.monotonic_ns()

    def start(self) -> None:
        self.follower.configure_follower()
        self.next_tick_ns = time.monotonic_ns()
        self._publish_status("waiting_for_command", command_topic=self.command_topic)
        LOGGER.info("进入 ROS2 话题控制状态，等待 %s。", self.command_topic)

    def step(self) -> None:
        now_ns = time.monotonic_ns()
        if now_ns < self.next_tick_ns:
            return
        self.next_tick_ns += self.period_ns
        if now_ns - self.next_tick_ns > self.period_ns:
            self.next_tick_ns = now_ns + self.period_ns
        try:
            _, follower_degrees = self.follower.read_positions_degrees()
            _, leader_degrees = self.leader.read_positions_degrees()
            self._publish_positions(self.follower_state_pub, follower_degrees)
            self._publish_positions(self.leader_state_pub, leader_degrees)
            if self.last_command is None or now_ns - self.last_command_ns > self.command_timeout_ns:
                if self.torque_enabled:
                    self.follower.disable_torque()
                    self.torque_enabled = False
                self._publish_status("command_timeout")
                return

            target = dict(zip(JOINT_NAMES, self.last_command))
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
