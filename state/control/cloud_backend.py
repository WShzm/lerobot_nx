"""云端转发控制的占位后端：不执行网络通信或电机命令。"""

from __future__ import annotations

import logging
from typing import Dict, Optional

from config import Config
from hardware.motors.feetech import FeetechArm

from .base import ControlBackend

LOGGER = logging.getLogger("state.control")


class CloudForwardControl(ControlBackend):
    """云端转发控制的占位后端。

    继承基类的状态发布、节拍与安全停机结构；`_next_target_degrees` 恒返回
    None，从臂保持扭矩失能。未来 WebSocket、MQTT、gRPC 或 HTTP 适配器
    （单命令或动作 chunk）应在本类实现 `_next_target_degrees`，需要观测
    输入时复用 hardware.camera.RealSenseFrames，而不是直接操作串口。
    """

    NODE_NAME = "so101_cloud_controller"
    idle_status_state = "cloud_reserved"

    def __init__(self, follower: FeetechArm, leader: FeetechArm, config: Config) -> None:
        super().__init__(follower, leader, config)
        LOGGER.warning("已选择云端转发控制，但该后端目前仅预留结构，保持从臂扭矩失能。")

    def _next_target_degrees(self, follower_degrees: Dict[str, float], now_ns: int) -> Optional[Dict[str, float]]:
        return None
