"""云端转发控制的占位后端，不执行网络通信或电机命令。"""

from __future__ import annotations

import logging

from hardware.motors.feetech import FeetechArm

from .base import ControlBackend

LOGGER = logging.getLogger("state.control")


class CloudForwardControl(ControlBackend):
    """云端转发控制的占位后端，不执行网络通信或电机命令。"""

    def __init__(self, follower: FeetechArm) -> None:
        self.follower = follower

    def start(self) -> None:
        self.follower.configure_follower()
        LOGGER.warning("已选择云端转发控制，但该后端目前仅预留结构，保持从臂扭矩失能。")

    def step(self) -> None:
        return

    def stop(self) -> None:
        self.follower.disable_torque()
