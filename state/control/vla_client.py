"""云端 VLA 服务的裸 TCP NDJSON 客户端：观测上行与 chunk 下行。

协议约定（两端一致，配合 fake_vla_server.py 作为参考实现）：
- 消息为单行 JSON（NDJSON），UTF-8 编码，"\n" 结尾；
- 观测端口（默认 8010）：NX 主动连接并逐行推送观测包，服务端只收不回；
- chunk 端口（默认 8001）：NX 主动连接后阻塞读行，服务端按自身频率推送；
- JPEG 在 JSON 内用 base64 编码，故消息体内不会出现换行，NDJSON 分帧安全。
"""

from __future__ import annotations

import json
import logging
import socket
import threading
from typing import Callable, Dict, Optional, Tuple
from urllib.parse import urlsplit

LOGGER = logging.getLogger("state.control.vla")

_RECONNECT_MAX_S = 5.0


def parse_url(url: str) -> Tuple[str, int]:
    """把 http://host:port 拆成 (host, port)；不合法直接抛 ValueError。"""
    parts = urlsplit(url)
    if parts.scheme != "http" or not parts.hostname or parts.port is None:
        raise ValueError("VLA 地址必须是 http://host:port 形式，收到 %r" % url)
    return parts.hostname, parts.port


def encode_line(payload: Dict[str, object]) -> bytes:
    return (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")


class ObsUploader:
    """向观测端口持续推送 NDJSON 行的客户端；连接断开时下次发送前惰性重连。"""

    def __init__(self, url: str) -> None:
        self.host, self.port = parse_url(url)
        self._sock = None  # type: Optional[socket.socket]
        self._lock = threading.Lock()

    def send(self, payload: Dict[str, object]) -> bool:
        line = encode_line(payload)
        with self._lock:
            if self._sock is None:
                sock = self._connect()
                if sock is None:
                    return False
                self._sock = sock
            try:
                self._sock.sendall(line)
                return True
            except OSError as exc:
                LOGGER.warning("观测包发送失败，下次发送前重连：%s", exc)
                self._close_locked()
                return False

    def _connect(self) -> Optional[socket.socket]:
        try:
            sock = socket.create_connection((self.host, self.port), timeout=3.0)
            sock.settimeout(None)
            LOGGER.info("已连接观测端口 %s:%d。", self.host, self.port)
            return sock
        except OSError as exc:
            LOGGER.info("观测端口 %s:%d 连接失败：%s", self.host, self.port, exc)
            return None

    def _close_locked(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def close(self) -> None:
        with self._lock:
            self._close_locked()


class ChunkSubscriber(threading.Thread):
    """连接 chunk 端口、逐行读取 NDJSON 的后台线程；断线按指数退避自动重连。"""

    def __init__(
        self,
        url: str,
        on_chunk: Callable[[Dict[str, object]], None],
        on_connected: Optional[Callable[[], None]] = None,
        on_disconnected: Optional[Callable[[], None]] = None,
    ) -> None:
        super().__init__(name="vla-chunk-subscriber", daemon=True)
        self.host, self.port = parse_url(url)
        self._on_chunk = on_chunk
        self._on_connected = on_connected
        self._on_disconnected = on_disconnected
        # 不能命名为 _stop：会遮蔽 threading.Thread 内部方法导致 join() 崩溃
        self._stop_event = threading.Event()

    def run(self) -> None:
        backoff_s = 1.0
        while not self._stop_event.is_set():
            try:
                sock = socket.create_connection((self.host, self.port), timeout=3.0)
            except OSError as exc:
                LOGGER.info("chunk 端口 %s:%d 连接失败，%.0f 秒后重试：%s", self.host, self.port, backoff_s, exc)
                if self._stop_event.wait(backoff_s):
                    return
                backoff_s = min(backoff_s * 2, _RECONNECT_MAX_S)
                continue
            backoff_s = 1.0
            LOGGER.info("已连接 chunk 端口 %s:%d，等待 chunk。", self.host, self.port)
            if self._on_connected is not None:
                self._on_connected()
            try:
                # 读超时 1 秒：让 stop() 能在秒级内打断阻塞的 readline。
                sock.settimeout(1.0)
                reader = sock.makefile("r", encoding="utf-8")
                while not self._stop_event.is_set():
                    try:
                        line = reader.readline()
                    except socket.timeout:
                        continue
                    if not line:
                        raise ConnectionError("chunk 连接被对端关闭")
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        message = json.loads(line)
                    except json.JSONDecodeError as exc:
                        LOGGER.warning("忽略无法解析的 chunk 行：%s", exc)
                        continue
                    try:
                        self._on_chunk(message)
                    except Exception:
                        LOGGER.exception("处理 chunk 消息时出错，已丢弃")
            except OSError as exc:
                LOGGER.warning("chunk 连接断开，准备重连：%s", exc)
            finally:
                try:
                    sock.close()
                except OSError:
                    pass
            if self._on_disconnected is not None:
                self._on_disconnected()
            if not self._stop_event.wait(backoff_s):
                backoff_s = min(backoff_s * 2, _RECONNECT_MAX_S)

    def stop(self) -> None:
        self._stop_event.set()
