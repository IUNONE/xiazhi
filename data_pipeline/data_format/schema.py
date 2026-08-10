from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import numpy as np

from data_pipeline.data_format.alignment import JOINT_NAMES


def build_lerobot_features(
    frame: Mapping[str, Any],
    *,
    use_videos: bool,
) -> dict[str, dict[str, Any]]:
    """Build a LeRobot 0.6 feature schema from one canonical paired frame."""
    features: dict[str, dict[str, Any]] = {}
    numeric_names = {
        "observation.state": list(JOINT_NAMES),
        "observation.ee_pose": ["x", "y", "z", "qw", "qx", "qy", "qz"],
        "observation.force": ["fx", "fy", "fz", "mx", "my", "mz"],
        "action": [f"{name}_target" for name in JOINT_NAMES],
    }
    for key, names in numeric_names.items():
        if key not in frame:
            continue
        value = np.asarray(frame[key], dtype=np.float32).reshape(-1)
        if value.size != len(names):
            raise ValueError(f"{key} 期望 {len(names)} 维，实际 {value.size} 维")
        features[key] = {
            "dtype": "float32",
            "shape": (value.size,),
            "names": names,
        }

    image_dtype = "video" if use_videos else "image"
    for key, value in frame.items():
        if not key.startswith("observation.images."):
            continue
        image = np.asarray(value)
        if image.dtype == np.uint16:
            depth = image
            if depth.ndim == 2:
                depth = depth[..., None]
            if depth.ndim != 3 or depth.shape[2] != 1 or depth.dtype != np.uint16:
                raise ValueError(f"{key} 必须是 HWC 单通道 uint16 毫米深度图")
            features[key] = {
                "dtype": image_dtype,
                "shape": tuple(depth.shape),
                "names": ["height", "width", "channels"],
                "info": {"is_depth_map": True, "depth_unit": "mm"},
            }
            continue
        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
            raise ValueError(f"{key} 必须是 HWC RGB uint8 或单通道 uint16 深度图")
        features[key] = {
            "dtype": image_dtype,
            "shape": tuple(image.shape),
            "names": ["height", "width", "channels"],
        }
    return features
