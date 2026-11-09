"""供原始采集器复用的 RealSense ROS2 图像订阅。"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
# 必须先于 cv_bridge 导入：cv_bridge_boost 的 boost::python 注册依赖 cv2 已加载，
# 否则以 "initialization of cv_bridge_boost raised unreported exception" 失败。
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


@dataclass(frozen=True)
class Frame:
    image: np.ndarray
    received_monotonic_ns: int
    ros_timestamp_ns: int


class RealSenseFrames(Node):
    """保存最新彩色和对齐深度帧，避免阻塞控制循环。"""

    def __init__(self, color_topic: str, depth_topic: Optional[str]) -> None:
        super().__init__("so101_raw_recorder")
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.color = None  # type: Optional[Frame]
        self.depth = None  # type: Optional[Frame]
        self.create_subscription(Image, color_topic, self._on_color, 10)
        if depth_topic:
            self.create_subscription(Image, depth_topic, self._on_depth, 10)
        self.get_logger().info("彩色图像话题：{}".format(color_topic))
        if depth_topic:
            self.get_logger().info("对齐深度图像话题：{}".format(depth_topic))

    @staticmethod
    def _ros_timestamp_ns(msg: Image) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _on_color(self, msg: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning("无法解码彩色图像：{}".format(exc))
            return
        frame = Frame(image.copy(), time.monotonic_ns(), self._ros_timestamp_ns(msg))
        with self.lock:
            self.color = frame

    def _on_depth(self, msg: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warning("无法解码深度图像：{}".format(exc))
            return
        if image.dtype != np.uint16:
            self.get_logger().warning("忽略深度图像类型 {}，期望 uint16".format(image.dtype))
            return
        frame = Frame(image.copy(), time.monotonic_ns(), self._ros_timestamp_ns(msg))
        with self.lock:
            self.depth = frame

    def latest(self) -> Tuple[Optional[Frame], Optional[Frame]]:
        with self.lock:
            return self.color, self.depth
