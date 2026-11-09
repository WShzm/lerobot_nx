"""默认控制后端：订阅 ROS2 单条关节命令话题。"""

from __future__ import annotations

import logging
import math
import time
from typing import Dict, List, Optional

from std_msgs.msg import Float64MultiArray

from config import Config
from hardware.motors.feetech import FeetechArm
from hardware.motors.joints import JOINT_NAMES

from .base import ControlBackend

LOGGER = logging.getLogger("state.control")


class RosTopicControl(ControlBackend):
    """默认控制后端：订阅单条关节命令，目标来源是最近一条合法命令。"""

    NODE_NAME = "so101_topic_controller"
    idle_status_state = "command_timeout"

    def __init__(self, follower: FeetechArm, leader: FeetechArm, config: Config) -> None:
        super().__init__(follower, leader, config)
        self.command_timeout_ns = int(config.command_timeout_s * 1e9)
        self.last_command = None  # type: Optional[List[float]]
        self.last_command_ns = 0
        self.create_subscription(Float64MultiArray, config.command_topic, self._on_command, 10)
        self.command_topic = config.command_topic

    def start(self) -> None:
        super().start()
        self._publish_status("waiting_for_command", command_topic=self.command_topic)
        LOGGER.info("进入 ROS2 话题控制状态，等待 %s。", self.command_topic)

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

    def _next_target_degrees(self, follower_degrees: Dict[str, float], now_ns: int) -> Optional[Dict[str, float]]:
        if self.last_command is None or now_ns - self.last_command_ns > self.command_timeout_ns:
            return None
        return dict(zip(JOINT_NAMES, self.last_command))
