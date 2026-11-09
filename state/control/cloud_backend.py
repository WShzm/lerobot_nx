"""云端 VLA 推理后端：观测组包上行、chunk 下行执行。"""

from __future__ import annotations

import base64
import logging
import math
import queue
import threading
import time
from collections import deque
from typing import Any, Callable, Deque, Dict, List, Optional

import cv2

from config import Config
from hardware.motors.joints import JOINT_NAMES

from .base import ControlBackend
from .vla_client import ChunkSubscriber, ObsUploader

LOGGER = logging.getLogger("state.control")


class CloudForwardControl(ControlBackend):
    """云端 VLA 推理后端，三个数据流与基类的关系：

    - 观测组包线程（生产者）：`_assemble_observation` 取最新图 + 广播读关节，
      过图龄门槛后放入有界队列，队列满时丢最旧，防止陈旧观测积压；
    - 上传线程（消费者）：队列出包，NDJSON 推到 VLA 观测端口；
    - chunk 订阅线程：长连 chunk 端口，校验后按到达时间展开成带截止时间的步进队列，
      append 策略，超过 max_pending_chunks 的新 chunk 直接丢弃。

    `_next_target_degrees` 按时间戳消费步进队列：步进耗尽但仍在
    inference_timeout_s 内时保持最后目标（位置保持），超时返回 None，
    基类失能从臂。限幅、扭矩生命周期、异常兜底全部由基类提供。
    """

    NODE_NAME = "so101_cloud_controller"
    idle_status_state = "inference_timeout"

    def __init__(self, follower: Any, leader: Any, config: Config, executor: Any) -> None:
        super().__init__(follower, leader, config)
        self._executor = executor
        self._camera = None
        self._obs_queue = queue.Queue(maxsize=int(config.max_obs_queue))  # type: queue.Queue
        self._steps = deque()  # type: Deque[tuple]  # 元素为 (deadline_ns, List[float])
        self._chunk_sizes = deque()  # type: Deque[int]  # 队列中各 chunk 的剩余步数
        self._steps_lock = threading.Lock()
        self._last_chunk_ns = 0
        self._last_target = None  # type: Optional[Dict[str, float]]
        self._stop = threading.Event()
        self._threads = []  # type: List[threading.Thread]
        self._uploader = ObsUploader(config.vla_obs_url)
        self._subscriber = ChunkSubscriber(
            config.vla_chunk_url,
            self._on_chunk,
            on_connected=self._on_chunk_connected,
            on_disconnected=self._on_chunk_disconnected,
        )

    # --- 生命周期 ---

    def start(self) -> None:
        super().start()
        self._camera = self._create_camera()
        if self._camera is not None:
            self._executor.add_node(self._camera)
        self._stop.clear()
        self._threads = [
            threading.Thread(target=self._obs_worker, name="vla-obs", daemon=True),
            threading.Thread(target=self._upload_worker, name="vla-upload", daemon=True),
        ]
        for thread in self._threads:
            thread.start()
        self._subscriber.start()
        LOGGER.info("云端 VLA 后端已启动：观测 → %s，chunk ← %s。", self.config.vla_obs_url, self.config.vla_chunk_url,)
        if self._camera is None:
            LOGGER.warning("未创建相机订阅，将不组装观测包。")
        self._publish_status("waiting_for_inference")

    def _create_camera(self) -> Any:
        # 惰性导入：cv_bridge 依赖仅云端后端需要，ros2 后端不加载。
        from hardware.camera import RealSenseFrames

        return RealSenseFrames(self.config.color_topic, None)

    def stop(self) -> None:
        self._stop.set()
        self._subscriber.stop()
        self._subscriber.join(timeout=3.0)
        for thread in self._threads:
            thread.join(timeout=3.0)
        self._uploader.close()
        if self._camera is not None:
            try:
                self._executor.remove_node(self._camera)
            except Exception:
                pass
            self._camera.destroy_node()
            self._camera = None
        with self._steps_lock:
            self._steps.clear()
            self._chunk_sizes.clear()
            self._last_target = None
        super().stop()

    # --- 观测组包（生产者）与上传（消费者）---

    def _assemble_observation(self) -> Optional[Dict[str, Any]]:
        """取最新图、广播读关节并打包；图像缺失或过龄时返回 None。"""
        if self._camera is None:
            return None
        color, _ = self._camera.latest()
        now_ns = time.monotonic_ns()
        if color is None or now_ns - color.received_monotonic_ns > int(self.config.max_image_age_ms * 1e6):
            return None
        _, follower_degrees = self.follower.read_positions_degrees()
        encoded, buffer = cv2.imencode(
            ".jpg", color.image, [cv2.IMWRITE_JPEG_QUALITY, int(self.config.jpeg_quality)]
        )
        if not encoded:
            LOGGER.warning("JPEG 编码失败，丢弃该观测。")
            return None
        return {
            "protocol_version": self.config.protocol_version,
            "robot_id": self.config.robot_id,
            "timestamp_ns": now_ns,
            "image_stamp_ns": color.received_monotonic_ns,
            "image_age_ms": (now_ns - color.received_monotonic_ns) / 1e6,
            "image_jpeg_b64": base64.b64encode(buffer.tobytes()).decode("ascii"),
            "joint_names": list(JOINT_NAMES),
            "joint_degrees": [follower_degrees[joint] for joint in JOINT_NAMES],
        }

    def _obs_worker(self) -> None:
        period_ns = int(1e9 / self.config.obs_sample_fps)
        next_ns = time.monotonic_ns()
        while not self._stop.is_set():
            now_ns = time.monotonic_ns()
            if now_ns < next_ns:
                self._stop.wait(max(0.001, (next_ns - now_ns) / 1e9))
                continue
            next_ns += period_ns
            if now_ns - next_ns > period_ns:
                next_ns = now_ns + period_ns
            package = None
            try:
                package = self._assemble_observation()
            except Exception:
                LOGGER.exception("组装观测包失败")
            if package is None:
                continue
            if self._obs_queue.full():
                try:
                    self._obs_queue.get_nowait()
                except queue.Empty:
                    pass
            self._obs_queue.put(package)

    def _upload_worker(self) -> None:
        while not self._stop.is_set():
            try:
                package = self._obs_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if self._uploader.send(package):
                self._publish_status("inferring")

    # --- chunk 接收、校验与展开 ---

    def _on_chunk_connected(self) -> None:
        self._publish_status("chunk_connected")

    def _on_chunk_disconnected(self) -> None:
        self._publish_status("chunk_disconnected")

    def _on_chunk(self, message: Dict[str, object]) -> None:
        actions = message.get("actions")
        if not isinstance(actions, list) or not actions:
            self._publish_status("invalid_chunk", reason="empty_actions")
            return
        protocol = message.get("protocol_version")
        if protocol != self.config.protocol_version:
            LOGGER.warning(
                "chunk protocol_version=%r 与本地 %r 不一致，仍按本地语义解析。",
                protocol,
                self.config.protocol_version,
            )
        rows = []  # type: List[List[float]]
        for row in actions:
            if not isinstance(row, list) or len(row) != len(JOINT_NAMES):
                self._publish_status("invalid_chunk", reason="bad_shape")
                return
            try:
                values = [float(value) for value in row]
            except (TypeError, ValueError):
                self._publish_status("invalid_chunk", reason="non_numeric")
                return
            if not all(math.isfinite(value) for value in values):
                self._publish_status("invalid_chunk", reason="non_finite")
                return
            rows.append(values)
        fps = message.get("fps")
        if not isinstance(fps, (int, float)) or fps <= 0:
            fps = self.config.model_fps
        exec_hint = message.get("exec_steps_hint")
        if isinstance(exec_hint, int) and exec_hint > 0:
            rows = rows[:exec_hint]
        elif self.config.exec_steps > 0:
            rows = rows[: self.config.exec_steps]

        now_ns = time.monotonic_ns()
        step_period_ns = int(1e9 / fps)
        with self._steps_lock:
            if len(self._chunk_sizes) >= self.config.max_pending_chunks:
                LOGGER.info("队列中已有 %d 个 chunk，丢弃新到的 chunk。", len(self._chunk_sizes))
                self._publish_status("chunk_dropped")
                return
            base_ns = self._steps[-1][0] if self._steps else now_ns
            for index, row in enumerate(rows):
                self._steps.append((base_ns + (index + 1) * step_period_ns, row))
            self._chunk_sizes.append(len(rows))
            self._last_chunk_ns = now_ns
        self._publish_status("chunk_received", chunk_id=message.get("chunk_id"))

    def _pop_expired_steps(self, now_ns: int) -> None:
        # 调用方持有 self._steps_lock。
        popped = 0
        while self._steps and self._steps[0][0] <= now_ns:
            self._steps.popleft()
            popped += 1
        while popped > 0 and self._chunk_sizes:
            take = min(popped, self._chunk_sizes[0])
            self._chunk_sizes[0] -= take
            popped -= take
            if self._chunk_sizes[0] == 0:
                self._chunk_sizes.popleft()

    # --- 基类钩子：60Hz 主循环消费动作步 ---

    def _next_target_degrees(self, follower_degrees: Dict[str, float], now_ns: int) -> Optional[Dict[str, float]]:
        with self._steps_lock:
            self._pop_expired_steps(now_ns)
            if self._steps:
                target = dict(zip(JOINT_NAMES, self._steps[0][1]))
                self._last_target = target
                return target
            # 步进耗尽：chunk 间隔内在最后目标上位置保持，超时后交给基类失能。
            if (
                self._last_target is not None
                and now_ns - self._last_chunk_ns <= int(self.config.inference_timeout_s * 1e9)
            ):
                return self._last_target
            return None
