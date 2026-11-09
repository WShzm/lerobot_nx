"""SO101 Xavier NX 启动配置"""

from __future__ import annotations

import logging
from pathlib import Path


class Config:

    leader_port = None  # 串口；None 仅从臂模式
    leader_calibration = None  # LeRobot 标定 JSON
    follower_port = "/dev/ttyACM0"
    follower_calibration = Path("~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_awesome_follower_arm.json")

    # --- 示教采集 ---
    output_root = Path("./data/so101_raw")  # episode 数据目录
    task = ""  # episode 元数据的任务描述
    fps = 30.0  # 示教与主从跟随频率
    color_topic = "/camera/camera/color/image_raw"
    depth_topic = "/camera/camera/aligned_depth_to_color/image_raw"
    record_depth = True  # False 只记录 RGB
    jpeg_quality = 92  # 彩色 JPEG 质量，[1, 100]
    max_image_age_ms = 250.0  # 允许图像帧的最大年龄

    # --- 从臂控制 ---
    control_backend = "ros2"  # "ros2/cloud"（云端预留）
    control_fps = 60.0  # 从臂控制频率
    command_topic = "/so101/control/command"  # 订阅：从臂位置控制
    follower_state_topic = "/so101/follower/state"  # 发布：从臂关节位置
    leader_state_topic = "/so101/leader/state"  # 发布：主臂关节位置
    action_topic = "/so101/control/action"  # 发布：安全限制后实际下发的动作
    status_topic = "/so101/status"  # 发布：JSON 格式状态
    command_timeout_s = 0.5  # 控制命令超时秒数
    max_relative_target_deg = 15.0  # 单周期最大目标变化角度

    # --- 云端 VLA 推理（control_backend = "cloud" 时生效）---
    vla_obs_url = "http://192.168.1.7:8010"  # 观测包上行目标（NDJSON over TCP）
    vla_chunk_url = "http://192.168.1.7:8001"  # chunk 下行来源（NDJSON over TCP）
    robot_id = "so101_nx_01"
    protocol_version = 1
    obs_sample_fps = 10.0  # 观测组包频率
    max_obs_queue = 4  # 观测有界队列长度，满时丢最旧
    model_fps = 30.0  # chunk 动作步默认间隔，服务端消息带 fps 时以消息为准
    exec_steps = 0  # 每 chunk 执行的前缀步数；0 = 全部执行，服务端 exec_steps_hint 优先
    max_pending_chunks = 2  # 队列中最多保留的 chunk 数（含正在执行的）
    inference_timeout_s = 3.0  # 超过该时长没有新 chunk 时失能从臂

    # --- 其他 ---
    skip_start_prompt = False  # True 跳过安全确认
    log_level = logging.INFO

    def __init__(self) -> None:
        self.follower_calibration = Path.expanduser(self.follower_calibration)
        if self.leader_calibration is not None:
            self.leader_calibration = Path.expanduser(self.leader_calibration)
        self._validate()

    def _validate(self) -> None:
        if (self.leader_port is None) != (self.leader_calibration is None):
            raise ValueError("leader_port 与 leader_calibration 必须同时设置或同时为 None")
        if self.fps <= 0 or self.control_fps <= 0:
            raise ValueError("fps 与 control_fps 必须为正数")
        if self.command_timeout_s <= 0:
            raise ValueError("command_timeout_s 必须为正数")
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("jpeg_quality 必须位于 [1, 100]")
        if self.max_image_age_ms <= 0:
            raise ValueError("max_image_age_ms 必须为正数")
        if self.control_backend not in ("ros2", "cloud"):
            raise ValueError("control_backend 只支持 ros2 或 cloud")
        if self.obs_sample_fps <= 0 or self.model_fps <= 0:
            raise ValueError("obs_sample_fps 与 model_fps 必须为正数")
        if self.inference_timeout_s <= 0:
            raise ValueError("inference_timeout_s 必须为正数")
        if self.max_obs_queue < 1 or self.max_pending_chunks < 1:
            raise ValueError("max_obs_queue 与 max_pending_chunks 必须 >= 1")
