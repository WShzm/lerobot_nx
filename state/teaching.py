"""主从示教状态使用的非阻塞原始数据采集会话。"""

from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Any, Dict, Optional

from rclpy.executors import Executor

from hardware.camera import RealSenseFrames
from hardware.motors.calibration import degrees_to_raw
from hardware.motors.feetech import FeetechArm
from hardware.motors.joints import JOINT_NAMES
from hardware.motors.safety import clip_goals

from .storage import EpisodeWriter

LOGGER = logging.getLogger("state.teaching")


def _as_joint_dict(values: Dict[str, Any]) -> Dict[str, Any]:
    return {joint: values[joint] for joint in JOINT_NAMES}


class TeachingSession:
    """示教状态的一个 episode，在主循环中通过 step 方法推进。"""

    def __init__(
        self,
        executor: Executor,
        leader: FeetechArm,
        follower: FeetechArm,
        output_root: Path,
        task: str,
        control_fps: float,
        color_topic: str,
        depth_topic: Optional[str],
        leader_calibration_path: Path,
        follower_calibration_path: Path,
        jpeg_quality: int,
        max_image_age_ms: float,
        max_relative_target_deg: float,
    ) -> None:
        self.executor = executor
        self.leader = leader
        self.follower = follower
        self.writer = EpisodeWriter(
            output_root,
            task,
            control_fps,
            color_topic,
            depth_topic,
            leader_calibration_path,
            follower_calibration_path,
            jpeg_quality,
        )
        self.color_topic = color_topic
        self.depth_topic = depth_topic
        self.control_fps = control_fps
        self.max_image_age_ns = int(max_image_age_ms * 1e6)
        self.max_relative_target_deg = max_relative_target_deg
        self.period_ns = int(1e9 / control_fps)
        self.camera = None  # type: Optional[RealSenseFrames]
        self.episode_dir = None  # type: Optional[Path]
        self.started_ns = 0
        self.next_tick_ns = 0
        self.count = 0
        self.torque_enabled = False
        self.last_frame_warning_ns = 0

    def start(self) -> Path:
        self.episode_dir = self.writer.open()
        self.camera = RealSenseFrames(self.color_topic, self.depth_topic)
        self.executor.add_node(self.camera)
        self.follower.configure_follower()
        self.started_ns = time.monotonic_ns()
        self.next_tick_ns = self.started_ns
        LOGGER.info("进入示教状态，正在等待相机帧：%s", self.episode_dir)
        return self.episode_dir

    def _warn_missing_frame(self, message: str) -> None:
        now_ns = time.monotonic_ns()
        if now_ns - self.last_frame_warning_ns > 1_000_000_000:
            LOGGER.warning(message)
            self.last_frame_warning_ns = now_ns

    def step(self) -> None:
        if self.camera is None or not self.writer.is_open:
            raise RuntimeError("示教采集会话尚未启动")
        now_ns = time.monotonic_ns()
        if now_ns < self.next_tick_ns:
            return
        self.next_tick_ns += self.period_ns
        if now_ns - self.next_tick_ns > self.period_ns:
            self.next_tick_ns = now_ns + self.period_ns

        color, depth = self.camera.latest()
        if color is None or now_ns - color.received_monotonic_ns > self.max_image_age_ns:
            self._warn_missing_frame("等待彩色图像帧，尚未使能从臂扭矩。")
            return
        if self.depth_topic is not None and (depth is None or now_ns - depth.received_monotonic_ns > self.max_image_age_ns):
            self._warn_missing_frame("等待对齐深度图像帧，尚未使能从臂扭矩。")
            return

        leader_raw, leader_degrees = self.leader.read_positions_degrees()
        follower_raw, follower_degrees = self.follower.read_positions_degrees()
        action_degrees = clip_goals(leader_degrees, follower_degrees, self.max_relative_target_deg)
        action_raw = {
            joint: degrees_to_raw(action_degrees[joint], self.follower.calibration[joint]) for joint in JOINT_NAMES
        }
        if not self.torque_enabled:
            self.follower.enable_torque()
            self.torque_enabled = True
        self.follower.write_goal_positions_raw(action_raw)

        rgb_meta, depth_meta = self.writer.queue_images(self.count, color, depth)
        sample = {
            "sample_index": self.count,
            "timestamp_ns": now_ns,
            "leader_position_deg": _as_joint_dict(leader_degrees),
            "leader_position_raw": _as_joint_dict(leader_raw),
            "action_position_deg": _as_joint_dict(action_degrees),
            "action_position_raw": _as_joint_dict(action_raw),
            "follower_position_deg": _as_joint_dict(follower_degrees),
            "follower_position_raw": _as_joint_dict(follower_raw),
            "rgb": rgb_meta,
        }
        if depth_meta is not None:
            sample["depth"] = depth_meta
        self.writer.append_sample(sample)
        self.count += 1

    def stop(self) -> None:
        try:
            self.follower.disable_torque()
        except Exception as exc:
            LOGGER.warning("结束示教时无法失能从臂扭矩：%s", exc)
        self.torque_enabled = False
        self.writer.close(num_samples=self.count, started_ns=self.started_ns)
        if self.camera is not None:
            self.executor.remove_node(self.camera)
            self.camera.destroy_node()
            self.camera = None
