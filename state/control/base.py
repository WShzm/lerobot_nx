"""从臂控制状态的统一生命周期接口。"""

from __future__ import annotations

from abc import ABC, abstractmethod


class ControlBackend(ABC):
    """从臂控制状态的统一生命周期接口。"""

    @abstractmethod
    def start(self) -> None:
        """进入从臂控制状态。"""

    @abstractmethod
    def step(self) -> None:
        """推进一次控制周期。"""

    @abstractmethod
    def stop(self) -> None:
        """退出从臂控制状态并使机械臂安全停机。"""
