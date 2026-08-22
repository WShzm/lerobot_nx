"""为 SO101 话题控制器提供经过校验的云端动作入口。

云端传输有意保持在硬件控制之外。WebSocket、gRPC、MQTT 或 NX 上的推理节点将
Float64MultiArray 动作发布到输入话题；本模块负责校验并转发到唯一由电机控制器消费的话题。
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from .constants import CLOUD_ACTION_TOPIC, CLOUD_STATUS_TOPIC, COMMAND_TOPIC, JOINT_NAMES

LOGGER = logging.getLogger("so101_nx.cloud_gateway")


class CloudActionGateway(Node):
    """在云端输出进入 SO101 控制话题前进行校验。"""

    def __init__(self, input_topic: str, output_topic: str, max_abs_degrees: float) -> None:
        super().__init__("so101_cloud_gateway")
        self.max_abs_degrees = max_abs_degrees
        self.output_pub = self.create_publisher(Float64MultiArray, output_topic, 10)
        self.status_pub = self.create_publisher(String, CLOUD_STATUS_TOPIC, 10)
        self.create_subscription(Float64MultiArray, input_topic, self._on_action, 10)
        self.publish_status("ready", input_topic=input_topic, output_topic=output_topic)

    def publish_status(self, state: str, **extra: object) -> None:
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

    def _on_action(self, message: Float64MultiArray) -> None:
        values = [float(value) for value in message.data]
        if len(values) != len(JOINT_NAMES):
            self.get_logger().error("拒绝包含 %d 个值的云端动作", len(values))
            self.publish_status("rejected", reason="wrong_length", received_values=len(values))
            return
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error("拒绝包含 NaN 或 Inf 的云端动作")
            self.publish_status("rejected", reason="non_finite")
            return
        if any(abs(value) > self.max_abs_degrees for value in values):
            self.get_logger().error("拒绝超出绝对角度限制的云端动作")
            self.publish_status("rejected", reason="absolute_limit")
            return
        output = Float64MultiArray()
        output.data = values
        self.output_pub.publish(output)
        self.publish_status("forwarded")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-topic", default=CLOUD_ACTION_TOPIC)
    parser.add_argument("--output-topic", default=COMMAND_TOPIC)
    parser.add_argument("--max-abs-degrees", type=float, default=180.0)
    parser.add_argument("--log-level", default="INFO", choices=("DEBUG", "INFO", "WARNING", "ERROR"))
    args = parser.parse_args()
    if args.max_abs_degrees <= 0:
        parser.error("--max-abs-degrees 必须为正数")
    logging.basicConfig(level=getattr(logging, args.log_level), format="%(asctime)s %(levelname)s %(message)s")
    rclpy.init(args=None)
    node = CloudActionGateway(args.input_topic, args.output_topic, args.max_abs_degrees)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        LOGGER.info("收到用户停止请求。")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
