from __future__ import annotations

import math
from typing import Any


class EndEffectorKinematics:
    """Use the ROS KDL solver with the same URDF as the viewer (radians/metres)."""

    def __init__(self, urdf: bytes, base_link: str, tip_link: str) -> None:
        try:
            import PyKDL
            from kdl_parser_py.urdf import treeFromString
        except ImportError as exc:
            raise ImportError(
                "EEF FK requires PyKDL and kdl_parser_py in the ROS Python environment"
            ) from exc
        valid, tree = treeFromString(urdf.decode("utf-8"))
        if not valid:
            raise ValueError("Cannot build KDL tree from robot URDF")
        chain = tree.getChain(base_link, tip_link)
        if chain.getNrOfJoints() == 0:
            raise ValueError(f"No movable joints between {base_link} and {tip_link}")
        self.names = []
        for i in range(chain.getNrOfSegments()):
            joint = chain.getSegment(i).getJoint()
            if joint.getTypeName() != "Fixed":
                self.names.append(joint.getName())
        self.kdl = PyKDL
        self.solver = PyKDL.ChainFkSolverPos_recursive(chain)

    def position(self, message: Any) -> tuple[float, float, float]:
        if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
            raise ValueError("Invalid JointState names/positions")
        values = dict(zip(message.name, message.position))
        missing = set(self.names) - values.keys()
        if missing:
            raise ValueError(f"JointState missing URDF joints: {sorted(missing)}")
        q = self.kdl.JntArray(len(self.names))
        for i, name in enumerate(self.names):
            value = float(values[name])
            if not math.isfinite(value):
                raise ValueError(f"Non-finite joint position: {name}")
            q[i] = value
        frame = self.kdl.Frame()
        if self.solver.JntToCart(q, frame) < 0:
            raise ValueError("KDL forward kinematics failed")
        return tuple(float(frame.p[i]) for i in range(3))
