"""主从示教状态使用的非阻塞原始数据采集会话。"""

from __future__ import annotations

import json
import logging
import shutil
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2
from rclpy.executors import Executor

from .calibration import degrees_to_raw
from .constants import JOINT_NAMES
from .feetech import FeetechArm
from .ros_camera import Frame, RealSenseFrames

LOGGER = logging.getLogger("so101_nx.teaching")


def _as_joint_dict(values: Dict[str, Any]) -> Dict[str, Any]:
    return {joint: values[joint] for joint in JOINT_NAMES}


def _clip_goals(
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


def _next_episode_dir(root: Path) -> Path:
    root.mkdir(parents=True, exist_ok=True)
    base = time.strftime("episode_%Y%m%d_%H%M%S")
    candidate = root / base
    sequence = 1
    while candidate.exists():
        candidate = root / "{}_{:02d}".format(base, sequence)
        sequence += 1
    candidate.mkdir()
    return candidate


def _write_json(path: Path, value: Dict[str, Any]) -> None:
    path.write_text(json.dumps(value, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")


def _save_frame(frame: Frame, path: Path, params: List[int]) -> None:
    if not cv2.imwrite(str(path), frame.image, params):
        raise RuntimeError("无法写入图像 {}".format(path))


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
        self.output_root = output_root
        self.task = task
        self.control_fps = control_fps
        self.color_topic = color_topic
        self.depth_topic = depth_topic
        self.leader_calibration_path = leader_calibration_path
        self.follower_calibration_path = follower_calibration_path
        self.jpeg_quality = jpeg_quality
        self.max_image_age_ns = int(max_image_age_ms * 1e6)
        self.max_relative_target_deg = max_relative_target_deg
        self.period_ns = int(1e9 / control_fps)
        self.camera = None  # type: Optional[RealSenseFrames]
        self.episode_dir = None  # type: Optional[Path]
        self.rgb_dir = None  # type: Optional[Path]
        self.depth_dir = None  # type: Optional[Path]
        self.samples_file = None
        self.metadata = {}  # type: Dict[str, Any]
        self.started_ns = 0
        self.next_tick_ns = 0
        self.count = 0
        self.torque_enabled = False
        self.last_frame_warning_ns = 0

    def start(self) -> Path:
        self.episode_dir = _next_episode_dir(self.output_root)
        self.rgb_dir = self.episode_dir / "rgb"
        self.depth_dir = self.episode_dir / "depth"
        self.rgb_dir.mkdir()
        if self.depth_topic is not None:
            self.depth_dir.mkdir()
        shutil.copy2(self.leader_calibration_path, self.episode_dir / "leader_calibration.json")
        shutil.copy2(self.follower_calibration_path, self.episode_dir / "follower_calibration.json")
        self.metadata = {
            "schema_version": 1,
            "status": "recording",
            "task": self.task,
            "started_wall_time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "control_fps": self.control_fps,
            "joint_names": list(JOINT_NAMES),
            "position_unit": "degrees",
            "raw_position_unit": "sts3215_encoder_ticks",
            "color_topic": self.color_topic,
            "depth_topic": self.depth_topic,
            "color_storage": "由 ROS bgr8 编码的 JPEG",
            "depth_storage": "来自 aligned_depth_to_color 的 16 位 PNG",
            "notes": "时间戳使用 NX 的 CLOCK_MONOTONIC；每张图像均保留 ROS 时间戳。",
        }
        _write_json(self.episode_dir / "episode.json", self.metadata)
        self.samples_file = (self.episode_dir / "samples.jsonl").open("a", encoding="utf-8", buffering=1)
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
        if self.camera is None or self.samples_file is None:
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
        action_degrees = _clip_goals(leader_degrees, follower_degrees, self.max_relative_target_deg)
        action_raw = {
            joint: degrees_to_raw(action_degrees[joint], self.follower.calibration[joint]) for joint in JOINT_NAMES
        }
        if not self.torque_enabled:
            self.follower.enable_torque()
            self.torque_enabled = True
        self.follower.write_goal_positions_raw(action_raw)

        if self.rgb_dir is None or self.episode_dir is None:
            raise RuntimeError("示教采集目录未初始化")
        rgb_name = "{:06d}_{}.jpg".format(self.count, color.received_monotonic_ns)
        _save_frame(color, self.rgb_dir / rgb_name, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        sample = {
            "sample_index": self.count,
            "timestamp_ns": now_ns,
            "leader_position_deg": _as_joint_dict(leader_degrees),
            "leader_position_raw": _as_joint_dict(leader_raw),
            "action_position_deg": _as_joint_dict(action_degrees),
            "action_position_raw": _as_joint_dict(action_raw),
            "follower_position_deg": _as_joint_dict(follower_degrees),
            "follower_position_raw": _as_joint_dict(follower_raw),
            "rgb": {
                "path": "rgb/{}".format(rgb_name),
                "received_monotonic_ns": color.received_monotonic_ns,
                "ros_timestamp_ns": color.ros_timestamp_ns,
            },
        }
        if depth is not None and self.depth_topic is not None:
            if self.depth_dir is None:
                raise RuntimeError("深度图像目录未初始化")
            depth_name = "{:06d}_{}.png".format(self.count, depth.received_monotonic_ns)
            _save_frame(depth, self.depth_dir / depth_name, [cv2.IMWRITE_PNG_COMPRESSION, 3])
            sample["depth"] = {
                "path": "depth/{}".format(depth_name),
                "received_monotonic_ns": depth.received_monotonic_ns,
                "ros_timestamp_ns": depth.ros_timestamp_ns,
            }
        self.samples_file.write(json.dumps(sample, ensure_ascii=True, separators=(",", ":")) + "\n")
        self.count += 1

    def stop(self) -> None:
        try:
            self.follower.disable_torque()
        except Exception as exc:
            LOGGER.warning("结束示教时无法失能从臂扭矩：%s", exc)
        self.torque_enabled = False
        if self.samples_file is not None:
            self.samples_file.close()
            self.samples_file = None
        if self.episode_dir is not None:
            self.metadata["status"] = "complete"
            self.metadata["ended_wall_time"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
            self.metadata["duration_s"] = (time.monotonic_ns() - self.started_ns) / 1e9
            self.metadata["num_samples"] = self.count
            _write_json(self.episode_dir / "episode.json", self.metadata)
            LOGGER.info("示教结束，已保存 %d 个样本：%s", self.count, self.episode_dir)
        if self.camera is not None:
            self.executor.remove_node(self.camera)
            self.camera.destroy_node()
            self.camera = None
