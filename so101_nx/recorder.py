"""主从遥操作与 RealSense 原始 episode 采集器。"""

from __future__ import annotations

import argparse
import json
import logging
import shutil
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2
import rclpy
from rclpy.executors import MultiThreadedExecutor

from .calibration import degrees_to_raw, load_calibration
from .constants import JOINT_NAMES
from .feetech import FeetechArm
from .ros_camera import Frame, RealSenseFrames

LOGGER = logging.getLogger("so101_nx.recording")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--leader-port", required=True)
    parser.add_argument("--follower-port", required=True)
    parser.add_argument("--leader-calibration", type=Path, required=True)
    parser.add_argument("--follower-calibration", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--episode-name")
    parser.add_argument("--task", default="")
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--duration-s", type=float)
    parser.add_argument("--color-topic", default="/camera/camera/color/image_raw")
    parser.add_argument("--depth-topic", default="/camera/camera/aligned_depth_to_color/image_raw")
    parser.add_argument("--no-depth", action="store_true")
    parser.add_argument("--jpeg-quality", type=int, default=92)
    parser.add_argument("--max-image-age-ms", type=float, default=250.0)
    parser.add_argument("--max-relative-target-deg", type=float, default=15.0)
    parser.add_argument("--no-control", action="store_true")
    parser.add_argument("--skip-start-prompt", action="store_true")
    parser.add_argument("--log-level", default="INFO", choices=("DEBUG", "INFO", "WARNING", "ERROR"))
    args = parser.parse_args()
    if args.fps <= 0:
        parser.error("--fps 必须为正数")
    if not 1 <= args.jpeg_quality <= 100:
        parser.error("--jpeg-quality 必须位于 [1, 100]")
    return args


def as_joint_dict(values: Dict[str, Any]) -> Dict[str, Any]:
    return {joint: values[joint] for joint in JOINT_NAMES}


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


def next_episode_dir(root: Path, requested_name: Optional[str]) -> Path:
    root.mkdir(parents=True, exist_ok=True)
    base = requested_name or time.strftime("episode_%Y%m%d_%H%M%S")
    candidate = root / base
    sequence = 1
    while candidate.exists():
        candidate = root / "{}_{:02d}".format(base, sequence)
        sequence += 1
    candidate.mkdir()
    return candidate


def write_json(path: Path, value: Dict[str, Any]) -> None:
    path.write_text(json.dumps(value, indent=2, ensure_ascii=True) + "\n", encoding="utf-8")


def save_frame(frame: Frame, path: Path, params: List[int]) -> None:
    if not cv2.imwrite(str(path), frame.image, params):
        raise RuntimeError("无法写入图像 {}".format(path))


def wait_for_frames(node: RealSenseFrames, want_depth: bool) -> None:
    LOGGER.info("正在等待 RealSense 图像帧...")
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        color, depth = node.latest()
        if color is not None and (not want_depth or depth is not None):
            return
        time.sleep(0.05)
    missing = "彩色和深度" if want_depth else "彩色"
    raise TimeoutError("15 秒内未收到{}图像帧，请检查 ROS 话题。".format(missing))


def record(args: argparse.Namespace) -> Path:
    leader_calibration = load_calibration(args.leader_calibration)
    follower_calibration = load_calibration(args.follower_calibration)
    episode_dir = next_episode_dir(args.output_root, args.episode_name)
    rgb_dir = episode_dir / "rgb"
    depth_dir = episode_dir / "depth"
    rgb_dir.mkdir()
    if not args.no_depth:
        depth_dir.mkdir()
    shutil.copy2(args.leader_calibration, episode_dir / "leader_calibration.json")
    shutil.copy2(args.follower_calibration, episode_dir / "follower_calibration.json")

    metadata = {
        "schema_version": 1,
        "status": "recording",
        "task": args.task,
        "started_wall_time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "control_fps": args.fps,
        "joint_names": list(JOINT_NAMES),
        "position_unit": "degrees",
        "raw_position_unit": "sts3215_encoder_ticks",
        "color_topic": args.color_topic,
        "depth_topic": None if args.no_depth else args.depth_topic,
        "color_storage": "由 ROS bgr8 编码的 JPEG",
        "depth_storage": "来自 aligned_depth_to_color 的 16 位 PNG",
        "notes": "时间戳使用 NX 的 CLOCK_MONOTONIC；每张图像均保留 ROS 时间戳。",
    }
    write_json(episode_dir / "episode.json", metadata)

    rclpy.init(args=None)
    frames = RealSenseFrames(args.color_topic, None if args.no_depth else args.depth_topic)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(frames)
    threading.Thread(target=executor.spin, name="ros2-executor", daemon=True).start()
    leader = FeetechArm(args.leader_port, leader_calibration)
    follower = FeetechArm(args.follower_port, follower_calibration)
    count = 0
    started_ns = 0
    try:
        wait_for_frames(frames, not args.no_depth)
        leader.connect()
        follower.connect()
        leader.verify_position_limits()
        follower.verify_position_limits()
        leader.disable_torque()
        follower.configure_follower()
        if not args.skip_start_prompt:
            input("已就绪。请将主臂置于安全姿态后按回车开始采集。")
        if not args.no_control:
            follower.enable_torque()

        started_ns = time.monotonic_ns()
        deadline_ns = None if args.duration_s is None else started_ns + int(args.duration_s * 1e9)
        period_ns = int(1e9 / args.fps)
        next_tick_ns = started_ns
        max_age_ns = int(args.max_image_age_ms * 1e6)
        LOGGER.info("正在采集 %s。按 Ctrl+C 正常结束。", episode_dir)
        with (episode_dir / "samples.jsonl").open("a", encoding="utf-8", buffering=1) as samples_file:
            while deadline_ns is None or time.monotonic_ns() < deadline_ns:
                now_ns = time.monotonic_ns()
                if now_ns < next_tick_ns:
                    time.sleep((next_tick_ns - now_ns) / 1e9)
                sample_ns = time.monotonic_ns()
                next_tick_ns += period_ns
                if sample_ns - next_tick_ns > period_ns:
                    next_tick_ns = sample_ns + period_ns

                color, depth = frames.latest()
                if color is None or sample_ns - color.received_monotonic_ns > max_age_ns:
                    LOGGER.warning("跳过当前样本：彩色图像不可用或已过期。")
                    continue
                if not args.no_depth and (depth is None or sample_ns - depth.received_monotonic_ns > max_age_ns):
                    LOGGER.warning("跳过当前样本：对齐深度图像不可用或已过期。")
                    continue

                leader_raw, leader_degrees = leader.read_positions_degrees()
                follower_raw, follower_degrees = follower.read_positions_degrees()
                action_degrees = clip_goals(leader_degrees, follower_degrees, args.max_relative_target_deg)
                action_raw = {
                    joint: degrees_to_raw(action_degrees[joint], follower_calibration[joint])
                    for joint in JOINT_NAMES
                }
                if not args.no_control:
                    follower.write_goal_positions_raw(action_raw)

                rgb_name = "{:06d}_{}.jpg".format(count, color.received_monotonic_ns)
                save_frame(color, rgb_dir / rgb_name, [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality])
                sample = {
                    "sample_index": count,
                    "timestamp_ns": sample_ns,
                    "leader_position_deg": as_joint_dict(leader_degrees),
                    "leader_position_raw": as_joint_dict(leader_raw),
                    "action_position_deg": as_joint_dict(action_degrees),
                    "action_position_raw": as_joint_dict(action_raw),
                    "follower_position_deg": as_joint_dict(follower_degrees),
                    "follower_position_raw": as_joint_dict(follower_raw),
                    "rgb": {
                        "path": "rgb/{}".format(rgb_name),
                        "received_monotonic_ns": color.received_monotonic_ns,
                        "ros_timestamp_ns": color.ros_timestamp_ns,
                    },
                }
                if depth is not None and not args.no_depth:
                    depth_name = "{:06d}_{}.png".format(count, depth.received_monotonic_ns)
                    save_frame(depth, depth_dir / depth_name, [cv2.IMWRITE_PNG_COMPRESSION, 3])
                    sample["depth"] = {
                        "path": "depth/{}".format(depth_name),
                        "received_monotonic_ns": depth.received_monotonic_ns,
                        "ros_timestamp_ns": depth.ros_timestamp_ns,
                    }
                samples_file.write(json.dumps(sample, ensure_ascii=True, separators=(",", ":")) + "\n")
                count += 1
    except KeyboardInterrupt:
        LOGGER.info("收到用户停止请求。")
    finally:
        try:
            follower.disable_torque()
        except Exception as exc:
            LOGGER.warning("结束时无法失能从臂扭矩：%s", exc)
        leader.close()
        follower.close()
        executor.shutdown()
        frames.destroy_node()
        rclpy.shutdown()

    metadata["status"] = "complete"
    metadata["ended_wall_time"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    metadata["duration_s"] = (time.monotonic_ns() - started_ns) / 1e9 if started_ns else 0.0
    metadata["num_samples"] = count
    write_json(episode_dir / "episode.json", metadata)
    LOGGER.info("已向 %s 保存 %d 个样本。", episode_dir, count)
    return episode_dir


def main() -> int:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level), format="%(asctime)s %(levelname)s %(message)s")
    try:
        record(args)
    except Exception as exc:
        LOGGER.exception("采集失败：%s", exc)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
