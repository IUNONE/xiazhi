from __future__ import annotations

from collections.abc import Iterable, Iterator, Mapping
from typing import Any

import numpy as np


JOINT_NAMES = tuple(f"joint_{index}" for index in range(1, 7))


def next_joint_target(
    next_observation: Mapping[str, Any],
    *,
    joint_count: int = 6,
) -> np.ndarray:
    """Build the deployment action for frame t from observation at frame t+1."""
    state = np.asarray(next_observation["observation.state"], dtype=np.float32).reshape(-1)
    if state.size < joint_count:
        raise ValueError(
            f"observation.state 只有 {state.size} 维，"
            f"无法构造 {joint_count} 维 joint target"
        )
    target = state[:joint_count].copy()
    if not np.isfinite(target).all():
        raise ValueError("下一状态包含 NaN/Inf，无法构造 action")
    return target


def pair_next_joint_targets(
    observations: Iterable[Mapping[str, Any]],
    *,
    joint_count: int = 6,
) -> Iterator[dict[str, Any]]:
    """Pair adjacent observations and deliberately drop the final observation."""
    iterator = iter(observations)
    try:
        previous = dict(next(iterator))
    except StopIteration:
        return
    for current in iterator:
        frame = dict(previous)
        frame["action"] = next_joint_target(current, joint_count=joint_count)
        yield frame
        previous = dict(current)
