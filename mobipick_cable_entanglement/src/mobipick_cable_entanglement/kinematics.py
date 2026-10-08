"""Forward kinematics from a URDF string, numpy only (no ROS, no KDL), so the cable model runs offline too."""
import xml.etree.ElementTree as ET
from typing import Dict, Iterable, List, Optional

import numpy as np


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_angle_to_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis
    c, s = np.cos(angle), np.sin(angle)
    C = 1.0 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def make_transform(R: np.ndarray, p: Iterable[float]) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


class _Joint:
    __slots__ = ("name", "type", "parent", "child", "origin", "axis", "lower", "upper")

    def __init__(self, name, jtype, parent, child, origin, axis, lower=-np.inf, upper=np.inf):
        self.name = name
        self.type = jtype
        self.parent = parent
        self.child = child
        self.origin = origin
        self.axis = axis
        self.lower = lower
        self.upper = upper


class UrdfKinematics:
    """Link poses of a URDF tree for a dict of joint positions (missing joints are at 0)."""

    def __init__(self, urdf_xml: str):
        root = ET.fromstring(urdf_xml)
        self._joint_by_child: Dict[str, _Joint] = {}
        self.joints: Dict[str, _Joint] = {}
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = [float(v) for v in o.get("xyz", "0 0 0").split()] if o is not None else [0.0, 0.0, 0.0]
            rpy = [float(v) for v in o.get("rpy", "0 0 0").split()] if o is not None else [0.0, 0.0, 0.0]
            a = j.find("axis")
            axis = np.array([float(v) for v in a.get("xyz").split()]) if a is not None else np.array([1.0, 0, 0])
            axis = axis / np.linalg.norm(axis)
            lim = j.find("limit")
            lower = float(lim.get("lower", -np.inf)) if lim is not None and j.get("type") != "continuous" else -np.inf
            upper = float(lim.get("upper", np.inf)) if lim is not None and j.get("type") != "continuous" else np.inf
            joint = _Joint(j.get("name"), j.get("type"), j.find("parent").get("link"), j.find("child").get("link"),
                           make_transform(rpy_to_matrix(*rpy), xyz), axis, lower, upper)
            self._joint_by_child[joint.child] = joint
            self.joints[joint.name] = joint
        self.links = [l.get("name") for l in root.findall("link")]

    def has_link(self, link: str) -> bool:
        return link in self.links

    def chain(self, link: str) -> List[_Joint]:
        """Joints from the root to `link`, root first."""
        out = []
        while link in self._joint_by_child:
            j = self._joint_by_child[link]
            out.append(j)
            link = j.parent
        return out[::-1]

    def root_of(self, link: str) -> str:
        ch = self.chain(link)
        return ch[0].parent if ch else link

    def joint_transform(self, joint: _Joint, q: float) -> np.ndarray:
        if joint.type in ("revolute", "continuous"):
            return joint.origin @ make_transform(axis_angle_to_matrix(joint.axis, q), (0, 0, 0))
        if joint.type == "prismatic":
            return joint.origin @ make_transform(np.eye(3), joint.axis * q)
        return joint.origin

    def joint_transform_rotation(self, joint: _Joint, q: float) -> np.ndarray:
        """Motion of the joint alone (without its fixed origin transform)."""
        if joint.type in ("revolute", "continuous"):
            return make_transform(axis_angle_to_matrix(joint.axis, q), (0, 0, 0))
        if joint.type == "prismatic":
            return make_transform(np.eye(3), joint.axis * q)
        return np.eye(4)

    def link_poses(self, links: Iterable[str], positions: Dict[str, float],
                   base: Optional[str] = None) -> Dict[str, np.ndarray]:
        """4x4 pose of every requested link in the frame of `base` (default: the URDF root)."""
        cache: Dict[str, np.ndarray] = {}

        def pose(link: str) -> np.ndarray:
            if link in cache:
                return cache[link]
            j = self._joint_by_child.get(link)
            T = np.eye(4) if j is None else pose(j.parent) @ self.joint_transform(j, positions.get(j.name, 0.0))
            cache[link] = T
            return T

        out = {l: pose(l) for l in links}
        if base is not None:
            Tb_inv = np.linalg.inv(pose(base))
            out = {l: Tb_inv @ T for l, T in out.items()}
        return out

    def joint_frame(self, joint_name: str, positions: Dict[str, float], base: Optional[str] = None):
        """(origin, unit axis) of a joint in the `base` frame: the axis the child link turns about."""
        j = self.joints[joint_name]
        Tp = self.link_poses([j.parent], positions, base)[j.parent] @ j.origin
        return Tp[:3, 3].copy(), Tp[:3, :3] @ j.axis
