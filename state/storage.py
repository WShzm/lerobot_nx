"""示教 episode 的目录创建、样本行与图像写入。"""

from __future__ import annotations

import json
import logging
import shutil
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2

from hardware.camera import Frame
from hardware.motors.joints import JOINT_NAMES

LOGGER = logging.getLogger("state.storage")


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


class EpisodeWriter:
    """管理一个 episode 的目录、元数据、样本行与图像文件，不参与控制逻辑。"""

    def __init__(
        self,
        output_root: Path,
        task: str,
        control_fps: float,
        color_topic: str,
        depth_topic: Optional[str],
        leader_calibration_path: Path,
        follower_calibration_path: Path,
        jpeg_quality: int,
    ) -> None:
        self.output_root = output_root
        self.task = task
        self.control_fps = control_fps
        self.color_topic = color_topic
        self.depth_topic = depth_topic
        self.leader_calibration_path = leader_calibration_path
        self.follower_calibration_path = follower_calibration_path
        self.jpeg_quality = jpeg_quality
        self.episode_dir = None  # type: Optional[Path]
        self.rgb_dir = None  # type: Optional[Path]
        self.depth_dir = None  # type: Optional[Path]
        self.samples_file = None
        self.metadata = {}  # type: Dict[str, Any]

    @property
    def is_open(self) -> bool:
        return self.samples_file is not None

    def open(self) -> Path:
        self.episode_dir = _next_episode_dir(self.output_root)
        self.rgb_dir = self.episode_dir / "rgb"
        self.rgb_dir.mkdir()
        if self.depth_topic is not None:
            self.depth_dir = self.episode_dir / "depth"
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
        return self.episode_dir

    def save_images(
        self, index: int, color: Frame, depth: Optional[Frame]
    ) -> Tuple[Dict[str, Any], Optional[Dict[str, Any]]]:
        """保存彩色和深度图像，返回样本行中的 rgb/depth 元数据。"""
        if self.rgb_dir is None or self.episode_dir is None:
            raise RuntimeError("示教采集目录未初始化")
        rgb_name = "{:06d}_{}.jpg".format(index, color.received_monotonic_ns)
        _save_frame(color, self.rgb_dir / rgb_name, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        rgb_meta = {
            "path": "rgb/{}".format(rgb_name),
            "received_monotonic_ns": color.received_monotonic_ns,
            "ros_timestamp_ns": color.ros_timestamp_ns,
        }
        if self.depth_topic is None or depth is None:
            return rgb_meta, None
        if self.depth_dir is None:
            raise RuntimeError("深度图像目录未初始化")
        depth_name = "{:06d}_{}.png".format(index, depth.received_monotonic_ns)
        _save_frame(depth, self.depth_dir / depth_name, [cv2.IMWRITE_PNG_COMPRESSION, 3])
        depth_meta = {
            "path": "depth/{}".format(depth_name),
            "received_monotonic_ns": depth.received_monotonic_ns,
            "ros_timestamp_ns": depth.ros_timestamp_ns,
        }
        return rgb_meta, depth_meta

    def append_sample(self, sample: Dict[str, Any]) -> None:
        if self.samples_file is None:
            raise RuntimeError("示教采集会话尚未启动")
        self.samples_file.write(json.dumps(sample, ensure_ascii=True, separators=(",", ":")) + "\n")

    def close(self, num_samples: int, started_ns: int) -> None:
        """关闭样本文件并把 episode 标记为完成。未打开时不做任何事。"""
        if self.samples_file is not None:
            self.samples_file.close()
            self.samples_file = None
        if self.episode_dir is not None:
            self.metadata["status"] = "complete"
            self.metadata["ended_wall_time"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
            self.metadata["duration_s"] = (time.monotonic_ns() - started_ns) / 1e9
            self.metadata["num_samples"] = num_samples
            _write_json(self.episode_dir / "episode.json", self.metadata)
            LOGGER.info("示教结束，已保存 %d 个样本：%s", num_samples, self.episode_dir)
