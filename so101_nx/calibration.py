"""兼容 LeRobot 的 SO101 标定文件读取与位置转换。"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict

from .constants import JOINT_NAMES, STS3215_RESOLUTION


@dataclass(frozen=True)
class Calibration:
    range_min: int
    range_max: int
    drive_mode: int


def load_calibration(path: Path) -> Dict[str, Calibration]:
    """读取由 LeRobot 为单个 SO101 机械臂生成的标定 JSON。"""
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError("无法读取标定文件 {}：{}".format(path, exc)) from exc

    calibration = {}
    for joint in JOINT_NAMES:
        value = data.get(joint)
        if not isinstance(value, dict):
            raise RuntimeError("{} 未包含 '{}' 的标定信息".format(path, joint))
        try:
            range_min = int(value["range_min"])
            range_max = int(value["range_max"])
            drive_mode = int(value.get("drive_mode", 0))
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError("{} 中 '{}' 的标定信息无效".format(path, joint)) from exc
        if range_min >= range_max:
            raise RuntimeError("{} 中 '{}' 的行程范围无效".format(path, joint))
        calibration[joint] = Calibration(range_min, range_max, drive_mode)
    return calibration


def raw_to_degrees(raw: int, calibration: Calibration) -> float:
    """使用与 LeRobot SO101 配置相同的角度归一化方式。"""
    midpoint = (calibration.range_min + calibration.range_max) / 2.0
    return (raw - midpoint) * 360.0 / STS3215_RESOLUTION


def degrees_to_raw(degrees: float, calibration: Calibration) -> int:
    midpoint = (calibration.range_min + calibration.range_max) / 2.0
    raw = round((degrees * STS3215_RESOLUTION / 360.0) + midpoint)
    return max(calibration.range_min, min(calibration.range_max, int(raw)))
