""": the cable carrier as a jointed chain in Gazebo (the gazebo_jointed_chain plugin), wired to the Mobipick.

Everything here is pure (URDF text, the cable model config, joint angles in; plugin SDF out), so it is tested without
Gazebo. Gazebo merges every link hung on a fixed joint into its moving ancestor (ur5_forearm_link, ur5_wrist_3_link,
the mobile base), so the clamps, the shapes and the chain's initial shape are expressed in those moving links.

The chain spans the wrist span of the cable model: from carrier_start to carrier_end, carrier_free_length long. The
shapes it collides with are the model's fitted shapes (config `capsules`), added to the robot's links at runtime; they
touch only the chain. Its initial shape is the model's settled carrier link chain at the joint angles of the spawn, so
it starts on the right side of every shape (a chain spawned through a shape stays on the wrong side of it Notes
2026-10-01 19:50).
"""
import math
from typing import Dict, List, Optional, Tuple
from xml.sax.saxutils import escape

import numpy as np

from .kinematics import UrdfKinematics, make_transform, rpy_to_matrix

# PLACEHOLDER values ( step 3, thinker notes 2026-10-01 19:12 and 20:03) until someone measures the real carrier:
# EI first (a cantilever of the real carrier with the cables in it, sag at 0.30 m), then the mass. The bend radius is
# not here: it comes from the cable model's link_chain disc sizes (bend_radius_from_config), so the Gazebo chain and
# the model describe the same carrier
CHAIN_DEFAULTS = {
    "links": 0,                  # 0 = the fewest links of at most max_link_length
    "max_link_length": 0.018,    # [m] at most about 30 deg per joint with the model's 3.5 cm bend radius (thinker 20:03)
    "bending_stiffness": 0.03,   # EI [N m^2] PLACEHOLDER
    "damping": 0.3,              # joint damping [N m s/rad] PLACEHOLDER
    "inertia_scale": 6.0,        # the smallest stable value with 12 g links ( step 1); lighter links: check
    "mass_per_metre": 0.35,      # [kg/m] PLACEHOLDER
    "friction": 0.3,             # the cable model's friction guess (cable_carrier.link_chain.friction)
    "collide_bitmask": 0x8000,
    "rate": 10.0,
    "transparency": 0.4,         # see-through, so the chain is told apart from the monitor's drawing of the span
}


def bend_radius_from_config(config: dict) -> float:
    """the carrier's minimum bend radius of the cable model: neighbouring discs touch at 2 atan(gap / diameter) per
    disc pitch (height + gap), the same numbers as cable_carrier.link_chain (16.3 deg per 1 cm = 3.5 cm)"""
    lc = config.get("cable_carrier", {}).get("link_chain", {})
    height, gap, diameter = (float(lc.get(k, v)) for k, v in
                             (("disc_height", 0.005), ("disc_gap", 0.005), ("disc_diameter", 0.035)))
    return (height + gap) / (2.0 * math.atan(gap / diameter))


def chain_links(length: float, chain: Optional[dict] = None) -> int:
    """the number of links the plugin gets for a chain of `length` (the same rule as plugin_sdf)"""
    c = dict(CHAIN_DEFAULTS, **(chain or {}))
    return int(c["links"]) or max(2, math.ceil(length / float(c["max_link_length"]) - 1e-9))


def arm_joint_state(names, positions, arm_joints) -> Optional[Dict[str, float]]:
    """{joint: angle} of one JointState message (`names`, `positions`) when it carries every joint of `arm_joints`,
    else None (a message of another part of the robot, e.g. only the gripper)"""
    q = {n: float(p) for n, p in zip(names, positions)}
    have = {n.split("/")[-1] for n in q}   # with or without the robot's prefix
    return q if all(j.split("/")[-1] in have for j in arm_joints) else None


def resample_equal(points: np.ndarray, n: int) -> np.ndarray:
    """n + 1 points at equal distances along a polyline: what the plugin does with the initial points"""
    points = np.asarray(points, dtype=float)
    along = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))])
    s = np.linspace(0.0, along[-1], n + 1)
    return np.stack([np.interp(s, along, points[:, i]) for i in range(3)], axis=1)


def joint_bends(points: np.ndarray) -> np.ndarray:
    """the angle between neighbouring segments at every inner point [rad]"""
    d = np.diff(points, axis=0)
    d = d / np.maximum(np.linalg.norm(d, axis=1), 1e-12)[:, None]
    return np.arccos(np.clip((d[:-1] * d[1:]).sum(axis=1), -1.0, 1.0))


def limit_bends(points: np.ndarray, n: int, max_bend: float, iterations: int = 2000, step: float = 0.03) -> np.ndarray:
    """`points` resampled to n links of equal length, with no joint bent more than `max_bend`: the plugin resamples
    the model's settled shape the same way, which cuts its corners and bends single joints beyond the model's own
    minimum bend radius (0.61 rad against 0.51 at the home pose), so the chain would start in its joint limits. A
    vertex above the limit moves a small step towards the middle of its neighbours (a big one shortens the chain),
    the ends stay in the clamps; the length is kept by resampling after every pass"""
    shape = resample_equal(points, n)
    for _ in range(iterations):
        bends = joint_bends(shape)
        over = np.nonzero(bends > max_bend)[0] + 1
        if len(over) == 0:
            return shape
        for i in over:
            shape[i] += step * ((shape[i - 1] + shape[i + 1]) / 2.0 - shape[i])
        shape = resample_equal(shape, n)
    raise ValueError(f"the carrier shape cannot be relaxed to {max_bend:.3f} rad per joint in {iterations} passes")


def moving_ancestor(kin: UrdfKinematics, link: str) -> Tuple[str, np.ndarray]:
    """(the first ancestor of `link` that is not hung on a fixed joint, the 4x4 pose of `link` in it): the link
    Gazebo merges `link` into"""
    T = np.eye(4)
    while True:
        j = kin._joint_by_child.get(link)
        if j is None or j.type != "fixed":
            return link, T
        T = j.origin @ T
        link = j.parent


def matrix_to_rpy(R: np.ndarray) -> Tuple[float, float, float]:
    """roll, pitch, yaw of a rotation matrix (R = Rz(yaw) Ry(pitch) Rx(roll), the URDF/SDF convention)"""
    pitch = float(np.arcsin(-np.clip(R[2, 0], -1.0, 1.0)))
    if abs(R[2, 0]) < 1.0 - 1e-9:
        return float(np.arctan2(R[2, 1], R[2, 2])), pitch, float(np.arctan2(R[1, 0], R[0, 0]))
    return float(np.arctan2(-R[1, 2], R[1, 1])), pitch, 0.0   # gimbal lock: yaw folded into roll


def pose_text(T: np.ndarray) -> str:
    return " ".join(f"{v:.6f}" for v in (*T[:3, 3], *matrix_to_rpy(T[:3, :3])))


def z_to(axis: np.ndarray) -> np.ndarray:
    """a rotation taking +z to `axis` (an SDF cylinder's axis is its z)"""
    z = axis / np.linalg.norm(axis)
    helper = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x = np.cross(helper, z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def anchor_in_gazebo(kin: UrdfKinematics, prefix: str, anchor: dict) -> Tuple[str, np.ndarray]:
    """(Gazebo link, position in it) of a config anchor {link, xyz}"""
    link, T = moving_ancestor(kin, prefix + anchor["link"])
    return link, (T @ np.append(np.asarray(anchor["xyz"], float), 1.0))[:3]


def shapes_in_gazebo(kin: UrdfKinematics, prefix: str, capsules: List[dict]) -> List[dict]:
    """the model's fitted shapes as plugin shapes ({name, link, type, pose, size}) on the Gazebo links. A flat-ended
    cylinder (a, b, radius, edge) becomes a cylinder, a box a box, a round-ended capsule a cylinder with a sphere at
    each end; the small edge rounding is left out"""
    out = []
    for c in capsules:
        link, T_link = moving_ancestor(kin, prefix + c["link"])
        if "box" in c:
            b = c["box"]
            T = T_link @ make_transform(rpy_to_matrix(*b["rpy"]), b["xyz"])
            out.append({"name": c["name"], "link": link, "type": "box", "pose": pose_text(T),
                        "size": " ".join(f"{v:.6f}" for v in b["size"])})
            continue
        a, b, r = np.asarray(c["a"], float), np.asarray(c["b"], float), float(c["radius"])
        length = float(np.linalg.norm(b - a))
        if length > 1e-6:
            T = T_link @ make_transform(z_to(b - a), (a + b) / 2.0)
            out.append({"name": c["name"], "link": link, "type": "cylinder", "pose": pose_text(T),
                        "size": f"{r:.6f} {length:.6f}"})
        if "edge" not in c or length <= 1e-6:   # a capsule: round ends
            for end, p in (("a", a), ("b", b)):
                out.append({"name": f"{c['name']}_{end}", "link": link, "type": "sphere",
                            "pose": pose_text(T_link @ make_transform(np.eye(3), p)), "size": f"{r:.6f}"})
    return out


def points_in_link(kin: UrdfKinematics, q_full: Dict[str, float], base: str, link: str,
                   points: np.ndarray) -> np.ndarray:
    """points given in the frame of `base` (the model's arm base) in the frame of `link` at the joint angles"""
    poses = kin.link_poses([base, link], q_full)
    T = np.linalg.inv(poses[link]) @ poses[base]
    return (T[:3, :3] @ np.asarray(points, float).T).T + T[:3, 3]


def wrist_span(spans: dict) -> dict:
    span = spans.get("wrist")
    if span is None:
        raise ValueError("cable model config: no wrist span")
    for key in ("carrier_start", "carrier_end", "carrier_free_length"):
        if key not in span:
            raise ValueError(f"cable model config: the wrist span has no {key}")
    return span


def plugin_sdf(kin: UrdfKinematics, config: dict, prefix: str, host_model: str, robot_namespace: str,
               chain: Optional[dict] = None, initial_points: Optional[np.ndarray] = None,
               stop_service: str = "", stop_strain: float = 0.0, stop_force: float = 0.0, stop_ticks: int = 3,
               holder_name: str = "cable_carrier_holder", chain_name: str = "cable_carrier") -> str:
    """the SDF of a static holder model carrying the plugin for the wrist span. `config` is the cable model config
    dict (cable_model.yaml), `prefix` the robot's link prefix (e.g. "mobipick/"), `initial_points` the chain's shape
    in the parent (carrier_start) Gazebo link, `chain` overrides of CHAIN_DEFAULTS"""
    c = dict(CHAIN_DEFAULTS, **(chain or {}))
    span = wrist_span(config["spans"])
    parent, pa = anchor_in_gazebo(kin, prefix, span["carrier_start"])
    child, pb = anchor_in_gazebo(kin, prefix, span["carrier_end"])
    length = float(span["carrier_free_length"])
    links = chain_links(length, chain)
    radius = float(config.get("cable_carrier_radius", 0.01725))
    reference, T_ref = moving_ancestor(kin, prefix + config.get("base_link", "ur5_base_link"))

    def tag(name, value):
        return f"<{name}>{escape(str(value))}</{name}>"

    body = [tag("chain_name", chain_name), tag("host_model", host_model), tag("robot_namespace", robot_namespace),
            tag("parent_link", parent), tag("parent_offset", " ".join(f"{v:.6f}" for v in pa) + " 0 0 0"),
            tag("child_link", child), tag("child_offset", " ".join(f"{v:.6f}" for v in pb) + " 0 0 0"),
            tag("links", links), tag("length", f"{length:.6f}"), tag("radius", f"{radius:.6f}"),
            tag("mass_per_link", f"{float(c['mass_per_metre']) * length / links:.6f}"),
            tag("inertia_scale", c["inertia_scale"]),
            tag("bend_radius", f"{float(c.get('bend_radius') or bend_radius_from_config(config)):.6f}"),
            tag("bending_stiffness", c["bending_stiffness"]), tag("damping", c["damping"]),
            tag("friction", c["friction"]), tag("collide_bitmask", int(c["collide_bitmask"])),
            tag("exclusive", "true"), tag("rate", c["rate"]), tag("transparency", c["transparency"]),
            tag("reference_link", reference), tag("reference_offset", pose_text(T_ref))]
    if stop_service:
        body += [tag("stop_service", stop_service), tag("stop_strain", stop_strain), tag("stop_force", stop_force),
                 tag("stop_ticks", stop_ticks)]
    if initial_points is not None and len(initial_points) >= 2:
        body.append(tag("initial_points", " ".join(f"{v:.5f}" for v in np.asarray(initial_points).ravel())))
    for s in shapes_in_gazebo(kin, prefix, config.get("capsules", [])):
        body.append("<shape>" + "".join(tag(k, s[k]) for k in ("name", "link", "type", "pose", "size")) + "</shape>")
    plugin = ('<plugin name="cable_carrier" filename="libgazebo_jointed_chain.so">\n      '
              + "\n      ".join(body) + "\n    </plugin>")
    # the holder is far below the floor and static: it only carries the plugin (the chain is its own model)
    return (f'<?xml version="1.0"?>\n<sdf version="1.6">\n  <model name="{escape(holder_name)}">\n'
            f'    <static>true</static>\n    <pose>0 0 -10 0 0 0</pose>\n'
            f'    <link name="anchor"><visual name="v"><geometry><sphere><radius>0.001</radius></sphere></geometry>'
            f'</visual></link>\n    {plugin}\n  </model>\n</sdf>\n')


def initial_points_from_model(model, q: Dict[str, float], config: dict, prefix: str,
                              chain: Optional[dict] = None) -> np.ndarray:
    """the cable model's carrier link chain of the wrist span, settled at the joint angles `q` (a fresh reset, as the
    monitor starts), in the Gazebo link of carrier_start: the chain's initial shape, resampled to the plugin's links
    with no joint beyond the plugin's bend limit (link length / bend radius)"""
    model.reset(q, settle_carrier=True)
    points = model.disc_points().get("wrist")
    if points is None or len(points) < 2:
        raise ValueError("the cable model has no carrier link chain for the wrist span (cable_carrier.enabled false?)")
    span = wrist_span(config["spans"])
    parent, _ = anchor_in_gazebo(model.kin, prefix, span["carrier_start"])
    points = points_in_link(model.kin, model._full_q(q), model.cfg.tf_prefix + model.cfg.base_link, parent, points)
    length = float(span["carrier_free_length"])
    links = chain_links(length, chain)
    radius = float((chain or {}).get("bend_radius") or bend_radius_from_config(config))
    return limit_bends(points, links, 0.97 * length / links / radius)
