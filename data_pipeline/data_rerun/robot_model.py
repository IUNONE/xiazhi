"""Expand the official robot xacro and resolve mesh URLs once per viewer."""
from __future__ import annotations

import os
import subprocess
from pathlib import Path

DEFAULT_ROBOT_XACRO = Path(__file__).resolve().parents[2] / "third_party/ros2_rm_robot/rm_description/urdf/rm_eco63.urdf.xacro"


def build_robot_urdf(robot_xacro: Path | None, link6_type: str = "Link6_6fb") -> bytes | None:
    if robot_xacro is None:
        return None
    if not robot_xacro.is_file():
        raise FileNotFoundError(f"机械臂 xacro 不存在: {robot_xacro}")

    xacro_executable = Path("/opt/ros/humble/bin/xacro")
    xacro_command = str(xacro_executable) if xacro_executable.is_file() else "xacro"
    xacro_environment = os.environ.copy()
    ros_python_paths = (
        Path("/opt/ros/humble/lib/python3.10/site-packages"),
        Path("/opt/ros/humble/local/lib/python3.10/dist-packages"),
    )
    python_paths = [str(path) for path in ros_python_paths if path.is_dir()]
    existing_python_path = xacro_environment.get("PYTHONPATH")
    if existing_python_path:
        python_paths.append(existing_python_path)
    if python_paths:
        xacro_environment["PYTHONPATH"] = os.pathsep.join(python_paths)
    try:
        completed = subprocess.run(
            [xacro_command, str(robot_xacro), f"link6_type:={link6_type}"],
            check=True,
            capture_output=True,
            text=True,
            env=xacro_environment,
        )
    except subprocess.CalledProcessError as exc:
        detail = exc.stderr.strip() or str(exc)
        raise RuntimeError(f"ECO63 xacro 展开失败: {detail}") from exc
    description_root = robot_xacro.parent.parent
    resource_prefix = description_root.as_uri() + "/"
    urdf = completed.stdout.replace("package://rm_description/", resource_prefix)
    return urdf.encode("utf-8")

