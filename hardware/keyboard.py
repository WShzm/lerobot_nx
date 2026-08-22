"""Linux 终端中的非阻塞单按键读取。"""

from __future__ import annotations

import select
import sys
import termios
import tty
from typing import Optional


class KeyboardReader:
    """在 cbreak 模式下读取单个按键，退出时恢复终端属性。"""

    def __init__(self) -> None:
        self.fd = None
        self.original_settings = None

    def __enter__(self) -> "KeyboardReader":
        if not sys.stdin.isatty():
            raise RuntimeError("需要可交互终端以接收 r、c 和 q 按键。")
        self.fd = sys.stdin.fileno()
        self.original_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        if self.fd is not None and self.original_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.original_settings)

    def read_key(self) -> Optional[str]:
        if self.fd is None:
            return None
        readable, _, _ = select.select([sys.stdin], [], [], 0)
        if not readable:
            return None
        return sys.stdin.read(1).lower()
