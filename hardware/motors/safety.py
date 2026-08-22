"""关节目标的安全限制策略。"""

from __future__ import annotations

from typing import Dict


def clip_goals(
    target_degrees: Dict[str, float], follower_degrees: Dict[str, float], max_delta_degrees: float
) -> Dict[str, float]:
    """把目标限制在当前位置 ±max_delta_degrees 以内，避免阶跃指令。

    按 target_degrees 的键迭代，两个字典应包含同一组关节；max_delta_degrees
    非正数时不限制。
    """
    if max_delta_degrees <= 0:
        return target_degrees
    return {
        joint: max(
            follower_degrees[joint] - max_delta_degrees,
            min(follower_degrees[joint] + max_delta_degrees, target_degrees[joint]),
        )
        for joint in target_degrees
    }
