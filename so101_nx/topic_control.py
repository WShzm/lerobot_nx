"""将本地或模型生成的 ROS2 关节命令执行到 SO101 的控制器。"""

from __future__ import annotations

import argparse
import json
import logging
import math
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from .calibration import degrees_to_raw, load_calibration
from .constants import (
    ACTION_TOPIC,
    COMMAND_TOPIC,
    FOLLOWER_STATE_TOPIC,
    JOINT_NAMES,
    LEADER_STATE_TOPIC,
    STATUS_TOPIC,
)
from .feetech import FeetechArm

LOGGER = logging.getLogger("so101_nx.topic_control")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--follower-port", required=True)
    parser.add_argument("--follower-calibration", type=Path, required=True)
    parser.add_argument("--leader-port", help="可选；提供后发布主臂状态")
    parser.add_argument("--leader-calibration", type=Path)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--command-topic", default=COMMAND_TOPIC)
    parser.add_argument("--command-timeout-s", type=float, default=0.5)
    parser.add_argument("--max-relative-target-deg", type=float, default=15.0)
    parser.add_argument("--skip-start-prompt", action="store_true")
    parser.add_argument("--log-level", default="INFO", choices=("DEBUG", "INFO", "WARNING", "ERROR"))
    args = parser.parse_args()
    if args.fps <= 0:
        parser.error("--fps 必须为正数")
    if args.command_timeout_s <= 0:
        parser.error("--command-timeout-s 必须为正数")
    if bool(args.leader_port) != bool(args.leader_calibration):
        parser.error("--leader-port 与 --leader-calibration 必须同时提供")
    return args


def clip_goals(
    target_degrees: Dict[str, float], follower_degrees: Dict[str, float], max_delta_degrees: float
) -> Dict[str, float]:
    if max_delta_degrees <= 0:
        return target_degrees
    return {
        joint: max(
            follower_degrees[joint] - max_delta_degrees,
            min(follower_degrees[joint] + max_delta_degrees, target_degrees[joint]),
        )
        for joint in JOINT_NAMES
    }


class SO101TopicController(Node):
    """执行角度空间命令，同时发布经过标定的关节状态。"""

    def __init__(
        self,
        follower: FeetechArm,
        leader: Optional[FeetechArm],
        command_topic: str,
        fps: float,
        command_timeout_s: float,
        max_relative_target_deg: float,
    ) -> None:
        super().__init__("so101_topic_controller")
        self.follower = follower
        self.leader = leader
        self.command_timeout_ns = int(command_timeout_s * 1e9)
        self.max_relative_target_deg = max_relative_target_deg
        self.last_command = None  # type: Optional[List[float]]
        self.last_command_ns = 0
        self.torque_enabled = False
        self.last_status = ""

        self.create_subscription(Float64MultiArray, command_topic, self._on_command, 10)
        self.follower_state_pub = self.create_publisher(Float64MultiArray, FOLLOWER_STATE_TOPIC, 10)
        self.leader_state_pub = self.create_publisher(Float64MultiArray, LEADER_STATE_TOPIC, 10)
        self.action_pub = self.create_publisher(Float64MultiArray, ACTION_TOPIC, 10)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.create_timer(1.0 / fps, self._on_timer)
        self.publish_status("waiting_for_command", command_topic=command_topic)

    def publish_status(self, state: str, **extra: object) -> None:
        payload = {
            "state": state,
            "joint_names": list(JOINT_NAMES),
            "position_unit": "degrees",
            "timestamp_ns": time.monotonic_ns(),
        }
        payload.update(extra)
        encoded = json.dumps(payload, separators=(",", ":"))
        if encoded != self.last_status:
            message = String()
            message.data = encoded
            self.status_pub.publish(message)
            self.last_status = encoded

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
            self.publish_status("invalid_command", received_values=len(message.data))
            return
        values = [float(value) for value in message.data]
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error("忽略包含 NaN 或 Inf 的命令")
            self.publish_status("invalid_command", reason="non_finite")
            return
        self.last_command = values
        self.last_command_ns = time.monotonic_ns()

    def _on_timer(self) -> None:
        try:
            _, follower_degrees = self.follower.read_positions_degrees()
            self._publish_positions(self.follower_state_pub, follower_degrees)
            if self.leader is not None:
                _, leader_degrees = self.leader.read_positions_degrees()
                self._publish_positions(self.leader_state_pub, leader_degrees)

            command_is_fresh = (
                self.last_command is not None
                and time.monotonic_ns() - self.last_command_ns <= self.command_timeout_ns
            )
            if not command_is_fresh:
                if self.torque_enabled:
                    self.follower.disable_torque()
                    self.torque_enabled = False
                self.publish_status("command_timeout")
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
            self.publish_status("executing", command_age_ms=(time.monotonic_ns() - self.last_command_ns) / 1e6)
        except Exception as exc:
            self.get_logger().error("控制器错误：%s", exc)
            try:
                self.follower.disable_torque()
            except Exception:
                pass
            self.torque_enabled = False
            self.publish_status("hardware_error", detail=str(exc))


def main() -> int:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level), format="%(asctime)s %(levelname)s %(message)s")
    follower = FeetechArm(args.follower_port, load_calibration(args.follower_calibration))
    leader = None
    if args.leader_port:
        leader = FeetechArm(args.leader_port, load_calibration(args.leader_calibration))
    rclpy.init(args=None)
    node = None
    try:
        follower.connect()
        follower.verify_position_limits()
        follower.configure_follower()
        if leader is not None:
            leader.connect()
            leader.verify_position_limits()
            leader.disable_torque()
        if not args.skip_start_prompt:
            input("已就绪。请确认从臂工作空间安全后按回车接收控制命令。")
        node = SO101TopicController(
            follower,
            leader,
            args.command_topic,
            args.fps,
            args.command_timeout_s,
            args.max_relative_target_deg,
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        LOGGER.info("收到用户停止请求。")
    except Exception as exc:
        LOGGER.exception("话题控制器失败：%s", exc)
        return 1
    finally:
        try:
            follower.disable_torque()
        except Exception:
            pass
        if node is not None:
            node.destroy_node()
        if leader is not None:
            leader.close()
        follower.close()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
