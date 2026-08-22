"""SO101 Xavier NX 的启动配置：命令行参数、参数校验与 ROS2 话题约定。"""

from __future__ import annotations

import argparse
from pathlib import Path

COMMAND_TOPIC = "/so101/control/command"
FOLLOWER_STATE_TOPIC = "/so101/follower/state"
LEADER_STATE_TOPIC = "/so101/leader/state"
ACTION_TOPIC = "/so101/control/action"
STATUS_TOPIC = "/so101/status"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="SO101 Xavier NX 统一入口：启动后进入从臂控制，按 r 示教、按 c 控制。"
    )
    parser.add_argument("--leader-port", required=True, help="主臂串口，例如 /dev/ttyACM0")
    parser.add_argument("--follower-port", required=True, help="从臂串口，例如 /dev/ttyACM1")
    parser.add_argument("--leader-calibration", type=Path, required=True, help="主臂 LeRobot 标定 JSON")
    parser.add_argument("--follower-calibration", type=Path, required=True, help="从臂 LeRobot 标定 JSON")
    parser.add_argument("--output-root", type=Path, default=Path("/data/so101_raw"), help="示教原始数据目录")
    parser.add_argument("--task", default="", help="写入每个 episode 元数据的任务描述")
    parser.add_argument("--fps", type=float, default=30.0, help="控制和采集频率")
    parser.add_argument(
        "--control-backend",
        choices=("ros2", "cloud"),
        default="ros2",
        help="启动时的从臂控制后端，默认 ros2；cloud 仅预留结构",
    )
    parser.add_argument("--command-topic", default=COMMAND_TOPIC, help="ROS2 控制命令话题")
    parser.add_argument("--command-timeout-s", type=float, default=0.5, help="控制命令超时秒数")
    parser.add_argument("--max-relative-target-deg", type=float, default=15.0, help="单周期最大目标变化角度")
    parser.add_argument("--color-topic", default="/camera/color/image_raw", help="RealSense 彩色图像话题")
    parser.add_argument(
        "--depth-topic",
        default="/camera/aligned_depth_to_color/image_raw",
        help="RealSense 对齐深度图像话题",
    )
    parser.add_argument("--no-depth", action="store_true", help="示教时不记录深度图像")
    parser.add_argument("--jpeg-quality", type=int, default=92, help="彩色 JPEG 质量")
    parser.add_argument("--max-image-age-ms", type=float, default=250.0, help="允许图像帧的最大年龄")
    parser.add_argument("--skip-start-prompt", action="store_true", help="跳过启动前的安全确认")
    parser.add_argument("--log-level", default="INFO", choices=("DEBUG", "INFO", "WARNING", "ERROR"))
    args = parser.parse_args()
    if args.fps <= 0:
        parser.error("--fps 必须为正数")
    if args.command_timeout_s <= 0:
        parser.error("--command-timeout-s 必须为正数")
    if not 1 <= args.jpeg_quality <= 100:
        parser.error("--jpeg-quality 必须位于 [1, 100]")
    if args.max_image_age_ms <= 0:
        parser.error("--max-image-age-ms 必须为正数")
    return args
