"""Mathematical model of the Mobipick arm cable (triflex cable carrier with the camera and F/T sensor cables).

The cable is clamped to the arm links (upper arm and forearm carriers, the terminal element on wrist_3). Between two
clamps it is free and crosses one or more joints: that is a *span*. For every span the model estimates the length
the cable needs in the current arm configuration, L_req, and compares it with the length it has, L_free:

    stretch s = L_req / L_free - 1 (s < 0: slack left, s >= 0: taut / over-stretched)
    tension T = EA * max(0, s) (the carrier and its clamps behave like a stiff spring once taut)
    torque tau_k = sum_spans T * dL_geo/dq_k (virtual work: the extra torque the UR motors feel)

L_req is the largest of these lower bounds:

1. L_geo, a function of the joint angles only: the shortest string from the carrier end A to the carrier end B around
   each crossed joint housing (a finite cylinder), see geometry.cylinder_geodesic. The winding angle uses the
   absolute (multi-turn) joint angle, so a wrist turned by more than half a turn stays wound.
2. The carrier link chain, history dependent: the carrier as rigid links carrying discs (CarrierLinkChains, C++
solver) hangs
   from its clamps under gravity and collides with the shapes of the arm, the gripper and the camera. Tightened and
   string-pulled, it gives the length the cable needs when its loop is caught behind the tool; a chain pulled longer
   than its links allow counts as that much stretch. (The radius rope, a free point-mass chain, was removed on
   2026-09-30: it caught none of the real cable stops.)

The model is observe-only: it reads joint positions, never commands the robot. See doc/cable_model.md.
"""
import ctypes
import math
from collections import OrderedDict
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from . import disc_native
from .geometry import cylinder_geodesic, cylindrical, polyline_length, string_pull
from .kinematics import UrdfKinematics

# (oscar 2026-10-01): the arm's cable guide is the CABLE CARRIER (it was tube, rope or disc chain). The old names
# of
# config keys, ROS parameter values and the Python API still work and warn once each; remove them after the real robot's
# configs are migrated. Kept on purpose: URDF/SRDF link names (cable_guide_*_tube_link), the disc sizes
# (disc_height/gap/diameter), the native solver (libdisc_chain), the real-robot log keys and the state fields (l_disc,
# disc_strain, disc_hooked_on).
_DEPRECATED_SEEN = set()


def deprecated_name(old: str, new: str) -> None:
    """warn once per process that the old name was used (rospy.logwarn in a node, else a FutureWarning)"""
    if old in _DEPRECATED_SEEN:
        return
    _DEPRECATED_SEEN.add(old)
    message = f"cable model: '{old}' is deprecated, use '{new}' (the arm's cable guide is the cable carrier)"
    try:
        import rospy
        if rospy.core.is_initialized():
            rospy.logwarn(message)
            return
    except Exception:  # noqa: BLE001 - no ROS
        pass
    import warnings
    warnings.warn(message, FutureWarning, stacklevel=3)

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
LEVELS = ["ok", "warning", "entangled"]


# ----------------------------------------------------------------------------------------------------------------
# configuration


@dataclass
class Anchor:
    link: str
    xyz: np.ndarray
    direction: np.ndarray  # unit vector (link frame) along which the cable leaves the clamp into the free span


@dataclass
class WrapJoint:
    joint: str
    radius: float
    extent: tuple  # (lo, hi) of the housing along the joint axis, joint frame [m]
    neutral: float  # joint angle [rad] at which the cable takes the short way around (fixes the winding branch)


@dataclass
class SpanConfig:
    name: str
    start: Anchor
    end: Anchor
    free_length: float
    lead: float
    joints: List[WrapJoint]
    stiffness: float  # EA [N]: tension per unit stretch once taut
    confidence: str = "medium"
    # the simulated carrier may start at another clamp point and have another length than the geometry that decides
    #; None = the same as start / free_length
    carrier_start: Optional[Anchor] = None
    # likewise the carrier's own tool end (the wrist carrier is clamped in the terminal element, not at the tip of
    # the URDF's guide link, which is MoveIt's mockup of the carrier); None = the same as end
    carrier_end: Optional[Anchor] = None
    carrier_free_length: Optional[float] = None
    # side on which the untangled carrier hangs (Anchor.direction in Anchor.link), None = straight down; see
    # ChainBase.reset
    carrier_rest_side: Optional[Anchor] = None
    # the deciding start at the real carrier support and its free length, used instead of start / free_length when
    # the config is built with wrist_anchor 'support' (see config_from_dict); None = the span has no such variant
    support_start: Optional[Anchor] = None
    support_free_length: Optional[float] = None

    @property
    def carrier_anchor(self) -> Anchor:
        return self.carrier_start or self.start

    @property
    def carrier_end_anchor(self) -> Anchor:
        return self.carrier_end or self.end

    @property
    def carrier_length(self) -> float:
        return self.carrier_free_length or self.free_length

    # the old attribute names, deprecated
    rope_start = property(lambda self: self._old("rope_start", "carrier_start"),
                          lambda self, v: self._old_set("rope_start", "carrier_start", v))
    rope_end = property(lambda self: self._old("rope_end", "carrier_end"),
                        lambda self, v: self._old_set("rope_end", "carrier_end", v))
    rope_free_length = property(lambda self: self._old("rope_free_length", "carrier_free_length"),
                                lambda self, v: self._old_set("rope_free_length", "carrier_free_length", v))
    rope_rest_side = property(lambda self: self._old("rope_rest_side", "carrier_rest_side"),
                              lambda self, v: self._old_set("rope_rest_side", "carrier_rest_side", v))
    rope_anchor = property(lambda self: self._old("rope_anchor", "carrier_anchor"))
    rope_end_anchor = property(lambda self: self._old("rope_end_anchor", "carrier_end_anchor"))
    rope_length = property(lambda self: self._old("rope_length", "carrier_length"))

    def _old(self, old: str, new: str):
        deprecated_name("SpanConfig." + old, "SpanConfig." + new)
        return getattr(self, new)

    def _old_set(self, old: str, new: str, value) -> None:
        deprecated_name("SpanConfig." + old, "SpanConfig." + new)
        setattr(self, new, value)


@dataclass
class Capsule:
    name: str
    link: str
    a: np.ndarray
    b: np.ndarray
    radius: float
    tool: bool = False  # part of the end effector: a carrier held there counts as "hooked"
    # None = a capsule (the segment a-b swept by a sphere of `radius`). A number = a flat-ended cylinder of
    # `radius` with its faces at a and b and edges rounded by `edge`: the core (a disc of radius - edge from
    # a + edge to b - edge along the axis) swept by a sphere of `edge`. Every consumer sees shapes as
    # (core_a, core_b, sweep, core_radius); a capsule has core_radius 0.
    edge: Optional[float] = None
    # a rounded box instead (gripper base, camera): centre, rotation (3 x 3, its columns are the box axes in
    # the link frame) and full size; a and b are its centre then. Core: the box shrunk by `edge` on every side.
    box_rot: Optional[np.ndarray] = None
    box_size: Optional[np.ndarray] = None

    def core(self):
        """(core_a, core_b, sweep radius, core radius, box: 3 core half sizes + rotation row-major, zeros if none) in
        the link frame"""
        if self.box_size is not None:
            e = min(self.edge or 0.0, 0.5 * float(np.min(self.box_size)))
            half = 0.5 * np.asarray(self.box_size, float) - e
            return self.a, self.a, e, 0.0, np.r_[np.maximum(half, 1e-9), np.asarray(self.box_rot, float).ravel()]
        if self.edge is None:
            return self.a, self.b, self.radius, 0.0, np.zeros(12)
        e = min(self.edge, self.radius)
        axis = self.b - self.a
        length = float(np.linalg.norm(axis))
        u = axis / length if length > 1e-12 else np.array([0.0, 0.0, 1.0])
        inset = min(e, 0.5 * length)
        return self.a + inset * u, self.b - inset * u, e, self.radius - e, np.zeros(12)


@dataclass
class LinkChainConfig:
    """: the carrier as a chain of rigid links on ball joints, each link carrying a disc; neighbouring discs touch
    at the largest joint angle. All sizes are approximations until the real carrier is measured."""
    disc_height: float = 0.005     # thickness of a disc along the carrier [m]
    disc_gap: float = 0.005        # free space between two discs on a straight carrier [m]
    disc_diameter: float = 0.035   # [m]
    # per update of max_dt (0.1 s); a shorter update takes fewer (see CarrierLinkChains.step)
    substeps: int = 100
    passes: int = 2                # constraint passes per substep
    gravity: float = 9.81          # [m/s^2]
    clamp_taper: float = 0.1       # see CarrierLinkChains.tapered_radius
    friction: float = 0.3          # Coulomb coefficient of the carrier on the arm and tool capsules (a guess)
    decide_on_strain: bool = True  # a chain pulled longer than its links allow counts as that much stretch,
    strain_tolerance: float = 0.01  # from this strain on (below it is the solver's own slack)
    twist_per_link_deg: float = 10.0  # torsion stop of one link (igus triflex R catalogue), logged only
    # a reset (node start, after a protective stop, the MTC guard's replay of every path) starts the chain hanging
    # below its chord and lets it settle; around some housings that lies on a side where the carrier does not fit (elbow
    # 114 deg with the upper arm up: 24 % strain, slack when the arm moved there). A chain still taut after that start
    # (strain >= strain_tolerance) is started again from the reference pose, where the cable counts as
    # untangled, and led along the straight joint path in steps of reset_sweep_deg; per chain the less taut
    # start is kept. 0 = off. A precaution since : with the shapes fitted to the meshes no named pose and no start
    # pose of a real cable stop is above 2 % strain without it (it was 24 % at elbow 114 deg with the old capsules).
    reset_sweep_deg: float = 3.0
    reset_settle_steps: int = 30   # updates of 0.1 s at the pose after the sweep
    # 2026-09-30 (oscar's live test): a reset settles the chain slowly (velocity kept per substep while it settles;
    # the normal motion keeps 1 - damping per 0.05 s): dropped from its start shape at full speed, the wrist loop swung
    # over the top of the wrist_1 cap and stayed there (+16.5 mm along the wrist_1 axis at home); settled slowly it
    # rests in front of the cap as on oscar's photo (-41 mm; the photo's band -38 .. -78 mm)
    settle_keep: float = 0.9
    # True: every reset starts the chain at the reference pose (home), where the untangled carrier's shape is known from
    # oscar's photo, and leads it to the pose along the straight joint path (reset_sweep_deg steps), the "untangled
    # way there" the twist already assumes; a chain that ends taut that way (e.g. home -> a place pose: +18 %) starts
    # at the pose itself if that is less taut. False: the chain starts at the pose itself (the sweep only when taut).
    # 2026-09-30, started at the pose: spawn -> home and transport -> home end in front of the wrist_1 cap, but
    # observe -> home is pulled taut (+9.3 %); from home: all three end in front of the cap, slack
    start_at_reference: bool = True
    # a reset reuses the chain settled at an earlier reset at (nearly) the same pose instead of settling it again
    # (the settle and the sweep from home are 1.4-1.5 s of the MTC guard's 2 s per path replay). Key: every joint
    # rounded to this step [mrad]; a hit starts from the cached chain and glides it to the exact pose in
    # reset_cache_glide_steps steps of 0.1 s (an exact repeat restores exactly the state a fresh reset gives).
    # 0 = off (the code default). ROS parameter cable_model/reset_cache_mrad overrides it (careful flag: a near
    # repeat can change a verdict).
    reset_cache_mrad: float = 0.0
    reset_cache_size: int = 2000
    reset_cache_glide_steps: int = 3

    @property
    def pitch(self) -> float:
        """joint to joint distance"""
        return self.disc_height + self.disc_gap

    @property
    def max_turn(self) -> float:
        """largest angle between two links [rad]: the rims of their discs touch (the joint sits in the middle of
        the gap, so tan(turn / 2) = (gap / 2) / (diameter / 2))"""
        return 2.0 * math.atan2(self.disc_gap, self.disc_diameter)

    @property
    def bend_radius(self) -> float:
        """radius of the carrier centre line with every joint at its stop"""
        return self.pitch / (2.0 * math.sin(self.max_turn / 2.0))


@dataclass
class CableCarrierConfig:
    """the simulated carrier (the carrier link chain, LinkChainConfig); enabled False = the geometric check L_geo
    alone"""
    enabled: bool = True
    segment_length: float = 0.03  # node spacing of the coarse copy whose holds give the chain's length
    substeps: int = 2             # with damping: the damping per time (CarrierLinkChains keeps it per its own substep)
    damping: float = 0.08
    contact_tolerance: float = 0.006
    tighten_iterations: int = 40
    evaluate_every: int = 3  # chain lengths are recomputed every n-th update (the chain itself steps every update)
    decide_on_topology: bool = False
    max_dt: float = 0.1
    link_chain: LinkChainConfig = field(default_factory=LinkChainConfig)

    @property
    def disc_link(self) -> LinkChainConfig:   # the old name, deprecated
        deprecated_name("rope.disc_link", "cable_carrier.link_chain")
        return self.link_chain

    @disc_link.setter
    def disc_link(self, value: LinkChainConfig) -> None:
        deprecated_name("rope.disc_link", "cable_carrier.link_chain")
        self.link_chain = value


# keys of the removed radius rope (2026-09-30); a config that still has them loads, their values are not used
_REMOVED_CARRIER_KEYS = ("iterations", "gravity", "bend_radius")


@dataclass
class DecisionConfig:
    warn_stretch: float = -0.04
    stop_stretch: float = 0.02
    stop_torque: Dict[str, float] = field(default_factory=dict)
    entangled_ticks: int = 2
    # 2026-09-30 (oscar: test the carrier link chain in isolation): False = the geometric string L_geo does not enter
    # the
    # decision while the chain runs (monitor, the MTC guard's path replay); evaluate() (the guard's geometric margin
    # stage, the servo guard, untangling) stays geometric. ROS parameter cable_model/geometric_check overrides it.
    geometric_check: bool = True


@dataclass
class ModelConfig:
    tf_prefix: str
    base_link: str
    cable_carrier_radius: float
    spans: List[SpanConfig]
    capsules: List[Capsule]
    reference: Dict[str, float]  # joint name (without prefix) -> rad
    cable_carrier: CableCarrierConfig
    decision: DecisionConfig
    extent_margin: float = 0.02
    wrist_anchor: str = "carrier_tip"   # which deciding start the spans use

    @property
    def rope(self) -> CableCarrierConfig:   # the old name, deprecated
        deprecated_name("ModelConfig.rope", "ModelConfig.cable_carrier")
        return self.cable_carrier

    @property
    def tube_radius(self) -> float:   # the old name, deprecated
        deprecated_name("ModelConfig.tube_radius", "ModelConfig.cable_carrier_radius")
        return self.cable_carrier_radius


def _vec(v) -> np.ndarray:
    return np.asarray(v, dtype=float)


def _anchor(d: dict) -> Anchor:
    direction = _vec(d.get("direction", [0, 0, 1]))
    return Anchor(d["link"], _vec(d.get("xyz", [0, 0, 0])), direction / np.linalg.norm(direction))


WRIST_ANCHORS = ("carrier_tip", "support", "auto")
_OLD_WRIST_ANCHORS = {"tube_tip": "carrier_tip"}   # , deprecated


def _new_anchor_name(choice):
    if choice in _OLD_WRIST_ANCHORS:
        deprecated_name(f"wrist_anchor {choice}", f"wrist_anchor {_OLD_WRIST_ANCHORS[choice]}")
        return _OLD_WRIST_ANCHORS[choice]
    return choice


def resolve_wrist_anchor(choice: str) -> str:
    """'carrier_tip' or 'support' for a wrist_anchor choice. 'auto': the global ROS parameter /cable_model/wrist_anchor if it names one, else 'support'
    with /use_sim_time and 'carrier_tip' without; 'carrier_tip' outside a ROS node (offline scripts and tests)."""
    choice = _new_anchor_name(choice)
    if choice not in WRIST_ANCHORS:
        raise ValueError(f"wrist_anchor {choice!r} is not one of {WRIST_ANCHORS}")
    if choice != "auto":
        return choice
    try:
        import rospy
        if not rospy.core.is_initialized():
            return "carrier_tip"
        # per robot first (<ns>/cable_model/wrist_anchor, relative to the node), then the sim-wide one
        override = rospy.get_param("cable_model/wrist_anchor", "") or rospy.get_param("/cable_model/wrist_anchor", "")
        override = _new_anchor_name(override)
        if override in ("carrier_tip", "support"):
            return override
        return "support" if rospy.get_param("/use_sim_time", False) else "carrier_tip"
    except Exception:  # noqa: BLE001 - no ROS: the calibrated default
        return "carrier_tip"


def _ros_param(name: str):
    """<node namespace>/cable_model/<name>, else /cable_model/<name>; None outside a ROS node or
    when neither is set"""
    try:
        import rospy
        if not rospy.core.is_initialized():
            return None
        for key in ("cable_model/" + name, "/cable_model/" + name):
            value = rospy.get_param(key, "")
            if value != "":
                return value
    except Exception:  # noqa: BLE001 - no ROS: the config's value
        pass
    return None


def _flag(value) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def span_enabled(name: str, span: dict) -> bool:
    """ (oscar 2026-09-30: only the wrist span ever entangled, the shoulder and elbow spans cost computation for
    nothing): the span's `enabled` in the config (default true), overridden by the ROS parameter
    cable_model/span_<name> (the monitor launch sets it from its args shoulder_span / elbow_span / wrist_span), so the
    monitor and the MTC cable guard switch together"""
    enabled = span.get("enabled", True)
    override = _ros_param("span_" + name)
    if override is not None:
        enabled = override
    if isinstance(enabled, str):
        enabled = enabled.strip().lower() in ("1", "true", "yes", "on")
    return bool(enabled)


def _carrier_config(d: dict) -> CableCarrierConfig:
    d = dict(d)
    disc = dict(d.pop("link_chain", None) or {})
    for key in ("disc_height", "disc_gap", "disc_diameter", "reset_cache_mrad"):
        override = _ros_param(key)
        if override is not None:
            disc[key] = float(override)
    bend_model = d.pop("bend_model", None)
    if bend_model not in (None, "disc_link"):
        raise ValueError(f"cable_carrier.bend_model {bend_model!r}: the radius rope model was removed (2026-09-30); "
                         "the carrier link chain is the only chain model. Delete the key; a running sim still holds "
                         "the old model "
                         "parameter: reload it (rosparam delete /mobipick/cable_entanglement_monitor/model, then "
                         "rosparam load config/cable_model.yaml into it) or start the bringup again")
    for key in _REMOVED_CARRIER_KEYS:
        d.pop(key, None)
    carrier = CableCarrierConfig(**d)
    carrier.link_chain = LinkChainConfig(**disc)
    return carrier


def _rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _shape(c: dict) -> "Capsule":
    """a `capsules:` entry: a capsule (a, b, radius), a flat cylinder (plus edge) or a rounded box (box: xyz, rpy,
    size; plus edge)"""
    edge = None if c.get("edge") is None else float(c["edge"])
    tool = bool(c.get("tool", False))
    if "box" in c:
        b = c["box"]
        centre = _vec(b["xyz"])
        return Capsule(c["name"], c["link"], centre, centre.copy(), 0.0, tool, edge,
                       _rpy_matrix(*[float(v) for v in b.get("rpy", [0, 0, 0])]), _vec(b["size"]))
    return Capsule(c["name"], c["link"], _vec(c["a"]), _vec(c["b"]), float(c["radius"]), tool, edge)


_OLD_KEYS = {"rope": "cable_carrier", "tube_radius": "cable_carrier_radius"}   # , deprecated
_OLD_SPAN_KEYS = {"rope_start": "carrier_start", "rope_end": "carrier_end", "rope_free_length": "carrier_free_length",
                  "rope_rest_side": "carrier_rest_side"}
_OLD_CARRIER_KEYS = {"disc_link": "link_chain"}


def _renamed(d: dict, names: Dict[str, str], where: str) -> dict:
    """a copy of d with the old keys under their new names, each warned once; the new key wins over an old one"""
    if not isinstance(d, dict) or not any(old in d for old in names):
        return d
    d = dict(d)
    for old, new in names.items():
        if old in d:
            deprecated_name(where + old, where + new)
            value = d.pop(old)
            d.setdefault(new, value)
    return d


def config_with_new_keys(d: dict) -> dict:
    """the config dict with the keys of before (rope:, tube_radius, rope_start/_end/_free_length/_rest_side,
    rope.disc_link, wrist_anchor tube_tip) under their new names; the dict passed in is not changed"""
    d = _renamed(d, _OLD_KEYS, "")
    if "cable_carrier" in d:
        d["cable_carrier"] = _renamed(d["cable_carrier"] or {}, _OLD_CARRIER_KEYS, "cable_carrier.")
    if isinstance(d.get("spans"), dict):
        d["spans"] = {name: _renamed(s, _OLD_SPAN_KEYS, "spans.<span>.") for name, s in d["spans"].items()}
    if d.get("wrist_anchor") in _OLD_WRIST_ANCHORS:
        d["wrist_anchor"] = _new_anchor_name(d["wrist_anchor"])
    return d


def config_from_dict(d: dict, wrist_anchor: Optional[str] = None, tf_prefix: Optional[str] = None) -> ModelConfig:
    """wrist_anchor (else the config's 'wrist_anchor', default 'carrier_tip'): 'support' makes every span with a
    support_start / support_free_length decide from there; see resolve_wrist_anchor for 'auto'. tf_prefix (else
    the config's, 'mobipick/'): the robot's link prefix, e.g. 'mobipick2/' for a second Mobipick in the sim.
    The keys of before the rename still load (config_with_new_keys, a warning each)."""
    d = config_with_new_keys(d)
    anchor = resolve_wrist_anchor(wrist_anchor or d.get("wrist_anchor", "carrier_tip"))
    spans = []
    for name, s in d["spans"].items():
        if not span_enabled(name, s):   # a switched-off span is not built or computed at all
            continue
        support = anchor == "support" and "support_start" in s and "support_free_length" in s
        spans.append(SpanConfig(
            name=name, start=_anchor(s["support_start"] if support else s["start"]), end=_anchor(s["end"]),
            free_length=float(s["support_free_length"] if support else s["free_length"]),
            lead=float(s.get("lead", 0.03)),
            joints=[WrapJoint(j["joint"], float(j["radius"]), tuple(float(v) for v in j.get("extent", [-1.0, 1.0])),
                              math.radians(float(j.get("neutral_deg", 0.0))))
                    for j in s.get("wrap_joints", [])],
            stiffness=float(s.get("stiffness", 2000.0)), confidence=str(s.get("confidence", "medium")),
            carrier_start=_anchor(s["carrier_start"]) if "carrier_start" in s else None,
            carrier_end=_anchor(s["carrier_end"]) if "carrier_end" in s else None,
            carrier_free_length=float(s["carrier_free_length"]) if "carrier_free_length" in s else None,
            carrier_rest_side=_anchor(s["carrier_rest_side"]) if "carrier_rest_side" in s else None,
            support_start=_anchor(s["support_start"]) if "support_start" in s else None,
            support_free_length=float(s["support_free_length"]) if "support_free_length" in s else None))
    capsules = [_shape(c) for c in d.get("capsules", [])]
    ref = {k: math.radians(float(v)) for k, v in d["reference_pose_deg"].items()}
    carrier = _carrier_config(d.get("cable_carrier", {}))
    dec = d.get("decision", {})
    decision = DecisionConfig(warn_stretch=float(dec.get("warn_stretch", -0.04)),
                              stop_stretch=float(dec.get("stop_stretch", 0.02)),
                              stop_torque={k: float(v) for k, v in dec.get("stop_torque", {}).items()},
                              entangled_ticks=int(dec.get("entangled_ticks", 2)),
                              geometric_check=_flag(dec.get("geometric_check", True)))
    override = _ros_param("geometric_check")
    if override is not None:
        decision.geometric_check = _flag(override)
    prefix = tf_prefix if tf_prefix is not None else d.get("tf_prefix", "mobipick/")
    return ModelConfig(tf_prefix=prefix.strip("/") + "/" if prefix.strip("/") else "", base_link=d.get("base_link", "ur5_base_link"),
                       cable_carrier_radius=float(d.get("cable_carrier_radius", 0.01725)), spans=spans,
                       capsules=capsules,
                       reference=ref, cable_carrier=carrier, decision=decision,
                       extent_margin=float(d.get("extent_margin", 0.02)), wrist_anchor=anchor)


# ----------------------------------------------------------------------------------------------------------------
# results


@dataclass
class SpanState:
    name: str
    chord: float
    l_geo: float
    l_req: float
    free_length: float
    stretch: float
    tension: float
    winding_deg: Dict[str, float]
    limiting: str  # what sets l_req: "chord", a joint name or "carrier link chain (...)"
    hooked_on: List[str]
    gradient: Dict[str, float]  # dL_req/dq per arm joint [m/rad]
    # the carrier link chain's own lengths and holds (None without the chain: cable_carrier.enabled false or a
    # failed update),
    # and the twist of the carrier between its clamps against the torsion stop (logged, never decides)
    l_disc: Optional[float] = None
    l_disc_topology: Optional[float] = None
    disc_hooked_on: Optional[List[str]] = None
    disc_strain: Optional[float] = None   # chain length over its nominal length minus one
    twist_deg: Optional[float] = None
    twist_limit_deg: Optional[float] = None


@dataclass
class CableState:
    spans: Dict[str, SpanState]
    torques: Dict[str, float]  # extra torque per arm joint [Nm]
    max_stretch: float
    worst_span: str
    predicted_stop: bool  # a protective stop is predicted for this tick (before hysteresis)
    level: str  # ok / warning / entangled (after hysteresis)
    reason: str

    def to_dict(self) -> dict:
        d = self._to_dict()
        for n, s in self.spans.items():
            if s.l_disc is not None:
                d["spans"][n]["disc"] = {
                    "l": round(s.l_disc, 4), "l_topology": round(s.l_disc_topology, 4), "hooked_on": s.disc_hooked_on,
                    "strain": round(s.disc_strain, 4),
                    "twist_deg": None if s.twist_deg is None else round(s.twist_deg, 1),
                    "twist_limit_deg": None if s.twist_limit_deg is None else round(s.twist_limit_deg, 1)}
        return d

    def _to_dict(self) -> dict:
        return {
            "level": self.level, "reason": self.reason, "predicted_stop": self.predicted_stop,
            "max_stretch": round(self.max_stretch, 4), "worst_span": self.worst_span,
            "torques": {k: round(v, 3) for k, v in self.torques.items()},
            "spans": {n: {
                "stretch": round(s.stretch, 4), "l_req": round(s.l_req, 4), "l_geo": round(s.l_geo, 4),
                "chord": round(s.chord, 4),
                "free_length": round(s.free_length, 4), "tension": round(s.tension, 2), "limiting": s.limiting,
                "winding_deg": {k: round(v, 1) for k, v in s.winding_deg.items()}, "hooked_on": s.hooked_on,
            } for n, s in self.spans.items()},
        }


# ----------------------------------------------------------------------------------------------------------------
# chain


def shape_records(caps) -> np.ndarray:
    """the shapes as the C++ code takes them: per shape 20 numbers, core ends a, b, sweep radius, core
    radius, box core half sizes and rotation (row-major; zeros unless a box)"""
    m = len(caps[2])
    core = caps[3] if len(caps) > 3 else np.zeros(m)
    box = caps[4] if len(caps) > 4 else np.zeros((m, 12))
    return np.ascontiguousarray(np.c_[caps[0], caps[1], caps[2], core, box], dtype=float)


def _norm(v: np.ndarray) -> np.ndarray:
    return np.sqrt(np.einsum("...k,...k->...", v, v))


def polyline_twist(x: np.ndarray, ref_start: np.ndarray, ref_end: np.ndarray) -> float:
    """Signed angle [rad] about the last segment from ref_start, carried along the polyline x without twisting
    (each step turns it by the smallest rotation between consecutive segment directions), to ref_end."""
    seg = np.diff(x, axis=0)
    length = _norm(seg)
    t = seg[length > 1e-9] / length[length > 1e-9][:, None]
    u = ref_start - (ref_start @ t[0]) * t[0]
    for a, b in zip(t[:-1], t[1:]):
        axis = np.cross(a, b)
        sin, cos = float(np.linalg.norm(axis)), float(a @ b)
        if sin > 1e-12:
            k = axis / sin
            u = u * cos + np.cross(k, u) * sin + k * (k @ u) * (1.0 - cos)  # Rodrigues
    end = t[-1]
    u = u - (u @ end) * end
    v = ref_end - (ref_end @ end) * end
    return math.atan2(float(np.cross(u, v) @ end), float(u @ v))


class ChainBase:
    """Chains of nodes between clamped carrier ends, one per span, in one node array: per chain the first and last two
    nodes are pinned (clamp point and one segment along the carrier direction), the others move. The start shape, the
    shapes of the arm and the tool (closest points, tightening) and the contacts; the physics is the subclass's
    step() (CarrierLinkChains; the radius rope that also lived here was removed on 2026-09-30)."""

    def __init__(self, lengths: List[float], cfg: CableCarrierConfig, segment_length: float):
        self.cfg = cfg
        self.chains = []  # (first node, node count, segment length)
        start = 0
        for length in lengths:
            n = max(8, int(round(length / segment_length)) + 1)
            self.chains.append((start, n, length / (n - 1)))
            start += n
        total = start
        self.x = np.zeros((total, 3))
        self.x_prev = np.zeros((total, 3))
        inv_mass = np.ones(total)
        for s0, n, _ in self.chains:
            inv_mass[[s0, s0 + 1, s0 + n - 2, s0 + n - 1]] = 0.0
        self.inv_mass = inv_mass
        self._free = np.nonzero(inv_mass > 0)[0]

    def chain(self, c: int) -> np.ndarray:
        s0, n, _ = self.chains[c]
        return self.x[s0:s0 + n]

    def seg(self, c: int) -> float:
        return self.chains[c][2]

    def _pins(self, pins):
        for (s0, n, seg), (a, ta, b, tb) in zip(self.chains, pins):
            self.x[s0], self.x[s0 + 1] = a, a + seg * ta
            self.x[s0 + n - 1], self.x[s0 + n - 2] = b, b + seg * tb

    def reset(self, pins, down, caps, settle_steps: int = 60, sides=None, shapes=None):
        """Start every chain untangled: hanging below its chord under gravity (a parabola with the sag its free
        length allows), pushed out of the housings (smoothing + collision, like tightened()) and released so that
        gravity settles it. Starting on the straight chord pushed the carrier out of the housings "the short way",
        which in observe_right led the elbow carrier over the top of the elbow, where the real one hangs below; a carrier cannot pass through the housings, so it stayed there.

        sides[c] (a unit vector or None) makes chain c sag towards that side of its chord instead of straight down,
        without the smoothing (it would pull the loop back onto the chord), and then drop under gravity: the wrist
        carrier hangs on the wrist_1 cap side, in front of the cap (Oscar's photo at home, 2026-09-28), where a start
        straight down went through the wrist housings and was pushed out over the top. A side start that ends with
        a segment through a housing falls back to the start straight down.

        shapes[c] (a polyline from clamp to clamp): chain c starts along it instead and only settles."""
        if shapes is not None:
            for (s0, n, _), shape in zip(self.chains, shapes):
                arc = np.r_[0.0, np.cumsum(_norm(np.diff(shape, axis=0)))]
                at = np.linspace(0.0, arc[-1], n)
                self.x[s0:s0 + n] = np.c_[[np.interp(at, arc, shape[:, k]) for k in range(3)]].T
            self._pins(pins)
            self._settle(pins, down, caps, settle_steps)
            return
        sides = sides or [None] * len(self.chains)
        for c, ((s0, n, seg), (a, ta, b, tb)) in enumerate(zip(self.chains, pins)):
            self.x[s0:s0 + n] = self._start_shape(n, seg, a, ta, b, tb, down, sides[c])
        self._pins(pins)
        for c in range(len(self.chains)):
            if sides[c] is None:
                s0, n, _ = self.chains[c]
                self.x[s0:s0 + n] = self.tightened(c, caps, 60)
        self._settle(pins, down, caps, settle_steps)
        through = [c for c in range(len(self.chains))
                   if sides[c] is not None and self.passes_through(self.chain(c), caps)]
        if through:
            for c in through:
                s0, n, seg = self.chains[c]
                self.x[s0:s0 + n] = self._start_shape(n, seg, *pins[c], down, None)
                self._pins(pins)
                self.x[s0:s0 + n] = self.tightened(c, caps, 60)
            self._settle(pins, down, caps, settle_steps)

    @staticmethod
    def _start_shape(n, seg, a, ta, b, tb, down, side):
        """Parabola of the chain's arc length between the lead points, sagging down, or towards side (its part
        across the chord)."""
        p0, p1 = a + seg * ta, b + seg * tb
        chord_vec = p1 - p0
        chord, length = float(np.linalg.norm(chord_vec)), seg * (n - 3)
        sag_dir = down
        if side is not None:
            across = side - (side @ chord_vec) * chord_vec / max(chord * chord, 1e-12)
            if np.linalg.norm(across) > 1e-6:
                sag_dir = across / np.linalg.norm(across)
        u = np.linspace(0.0, 1.0, n - 2)
        sag = math.sqrt(max(0.0, 3.0 * chord * (length - chord) / 8.0))  # parabola of that arc length
        x = np.empty((n, 3))
        x[1:n - 1] = p0 + np.outer(u, chord_vec) + np.outer(4.0 * sag * u * (1.0 - u), sag_dir)
        x[0], x[-1] = a, b
        return x

    def _settle(self, pins, down, caps, steps):
        self.x_prev = self.x.copy()
        for _ in range(steps):
            self.step(0.05, pins, down, caps)
        self.x_prev = self.x.copy()

    @staticmethod
    def passes_through(x, caps, tol: float = 0.01) -> bool:
        """True when a segment midpoint of the chain lies inside a capsule (the chain went through it). The three
        segments at each clamp are left out: the clamps sit on the arm surface (the shapes fitted to the meshes
        lie close to the forearm carrier clamp, and the first 3 cm segment grazed one by 7 mm at home). tol 1 cm: a 3 cm segment whose ends rest on a flat end cuts up to 7.3 mm into its padded rounded edge; a
        chain through a housing goes much deeper."""
        mid = 0.5 * (x[3:-4] + x[4:-3])
        if len(mid) == 0:
            return False
        _, _, _, pen = ChainBase.closest(mid, caps)
        return bool((pen.min(axis=1) < -tol).any())

    def step(self, dt, pins, down, caps):
        raise NotImplementedError

    @staticmethod
    def closest(p, caps):
        """Closest core points (N,M,3), offsets, distances and penetration (distance - sweep radius). The core of a
        capsule is its axis; a flat cylinder's core is a disc of core radius caps[3] along its axis: its
        closest point clamps the offset from the axis to that radius, and a point inside the core gets the nearest
        point of the core's surface with an offset of 1e-9 m pointing out (so c + offset / |offset| * radius, what
        the callers do, puts it on the surface)."""
        ca, cb, cr = caps[:3]
        ab = cb - ca
        t = np.einsum("nmk,mk->nm", p[:, None, :] - ca[None], ab) / np.maximum(np.einsum("mk,mk->m", ab, ab), 1e-12)
        c = ca[None] + np.clip(t, 0.0, 1.0)[..., None] * ab[None]
        d = p[:, None, :] - c
        if len(caps) > 3 and np.any(caps[3] > 0.0):
            c, d = ChainBase._core_closest(p, ca, ab, caps[3], t, c, d)
        if len(caps) > 4 and np.any(caps[4][:, 0] > 0.0):
            c, d = ChainBase._box_closest(p, ca, caps[4], c, d)
        dist = _norm(d)
        return c, d, dist, dist - cr[None]

    @staticmethod
    def _box_closest(p, centre, box, c, d):
        """closest() for the rounded boxes: the nearest point of the core box (a point inside goes out through the
        nearest face, offset 1e-9 m along its normal)"""
        cols = np.where(box[:, 0] > 0.0)[0]
        half = box[cols, :3]
        rot = box[cols, 3:].reshape(-1, 3, 3)                        # columns: the box axes
        local = np.einsum("mki,nmk->nmi", rot, p[:, None, :] - centre[cols][None])
        q = np.clip(local, -half[None], half[None])
        inside = np.all(np.abs(local) < half[None], axis=-1)
        cc = centre[cols][None] + np.einsum("mik,nmk->nmi", rot, q)
        dd = p[:, None, :] - cc
        if inside.any():
            gap = half[None] - np.abs(local)                           # to each face pair
            ax = np.argmin(gap, axis=-1)
            sign = np.sign(np.take_along_axis(local, ax[..., None], -1))[..., 0]
            sign = np.where(sign == 0.0, 1.0, sign)
            n_local = np.zeros_like(local)
            np.put_along_axis(n_local, ax[..., None], sign[..., None], -1)
            n_out = np.einsum("mik,nmk->nmi", rot, n_local)
            surf = p[:, None, :] + np.take_along_axis(gap, ax[..., None], -1) * n_out
            cc = np.where(inside[..., None], surf, cc)
            dd = np.where(inside[..., None], n_out * 1e-9, dd)
        c, d = c.copy(), d.copy()
        c[:, cols], d[:, cols] = cc, dd
        return c, d

    @staticmethod
    def _core_closest(p, ca, ab, core_r, t, c, d):
        """closest() for the shapes with a core radius (flat cylinders), in place of the axis point"""
        cols = np.where(core_r > 0.0)[0]
        length = np.sqrt(np.einsum("mk,mk->m", ab[cols], ab[cols]))
        u = ab[cols] / np.maximum(length, 1e-12)[:, None]
        tt = t[:, cols]
        axial = ca[cols][None] + np.clip(tt, 0.0, 1.0)[..., None] * ab[cols][None]
        w = p[:, None, :] - axial
        w = w - np.einsum("nmk,mk->nm", w, u)[..., None] * u[None]      # the part across the axis
        rho = _norm(w)
        R = core_r[cols][None]
        w_hat = w / np.maximum(rho, 1e-12)[..., None]
        cc = axial + np.minimum(rho, R)[..., None] * w_hat
        dd = p[:, None, :] - cc
        inside = (rho < R) & (tt > 0.0) & (tt < 1.0)
        if inside.any():
            s = tt * length[None]                           # distance from face a along the axis
            depth = np.stack([R - rho, s, length[None] - s], -1)
            which = np.argmin(depth, axis=-1)
            face = np.where(which == 1, -1.0, 1.0)[..., None] * u[None]
            n_out = np.where((which == 0)[..., None], w_hat, face)
            to_surface = np.take_along_axis(depth, which[..., None], -1)
            surf = p[:, None, :] + to_surface * n_out
            cc = np.where(inside[..., None], surf, cc)
            dd = np.where(inside[..., None], n_out * 1e-9, dd)
        c, d = c.copy(), d.copy()
        c[:, cols], d[:, cols] = cc, dd
        return c, d

    def tightened(self, c: int, caps, iterations: int) -> np.ndarray:
        """Copy of chain c pulled towards its shortest shape while the capsules keep it in its topology.

        Red-black Gauss-Seidel Laplacian smoothing (every free node moves to the midpoint of its neighbours) with
        collision projection after each half step. Points that merely rest on a housing slide off; points where
        the chain is really held (wound around a housing, hooked behind the tool) stay in contact.
        """
        s0, n, _ = self.chains[c]
        return self.tighten(self.x[s0:s0 + n].copy(), caps, iterations)

    @staticmethod
    def tighten(x: np.ndarray, caps, iterations: int) -> np.ndarray:
        if disc_native.available():   # the same in C++ (one call instead of ~100 closest-point searches)
            x = np.ascontiguousarray(x, dtype=float)
            flat = shape_records(caps)
            as_d = lambda arr: arr.ctypes.data_as(ctypes.POINTER(ctypes.c_double))  # noqa: E731
            disc_native.load().rope_tighten(as_d(x), len(x), as_d(flat), len(flat), int(iterations))
            return x
        n = len(x)
        even = np.arange(2, n - 2, 2)
        odd = np.arange(3, n - 2, 2)
        for _ in range(iterations):
            for idx in (even, odd):
                if len(idx) == 0:
                    continue
                p = 0.5 * (x[idx - 1] + x[idx + 1])
                cc, d, dist, pen = ChainBase.closest(p, caps)
                k = np.argmin(pen, axis=1)
                rows = np.arange(len(p))
                inside = pen[rows, k] < 0.0
                if inside.any():
                    r, kk = rows[inside], k[inside]
                    nrm = d[r, kk] / np.maximum(dist[r, kk], 1e-9)[:, None]
                    p[r] = cc[r, kk] + nrm * caps[2][kk][:, None]
                x[idx] = p
        return x

    def contacts(self, c: int, caps, tol):
        """(node, capsule index) of every free node of chain c touching a capsule (nearest capsule)."""
        s0, n, _ = self.chains[c]
        idx = np.arange(s0 + 2, s0 + n - 2)
        if len(idx) == 0:
            return []
        _, _, _, pen = self.closest(self.x[idx], caps)
        k = np.argmin(pen, axis=1)
        touch = pen[np.arange(len(idx)), k] < tol
        return list(zip(idx[touch].tolist(), k[touch].tolist()))


class CarrierLinkChains(ChainBase):
    """: the carrier as a chain of rigid links on ball joints, each link carrying a disc; the discs of two links
    touch at the largest joint angle (LinkChainConfig). One chain per span, with the carrier's links as segments.

    A chain of many short links with hard stops needs its own solver (native/disc_chain.cpp, compiled: the numpy
    version took 44 ms per update and still let the chain stretch):

    - many small substeps with the real gravity: projected pair by pair at a coarse step (0.05 s), a chain stays
      stretched under its own weight (the removed 3 cm radius rope hung about 20 % longer than its length); the
      error falls with the square of the substep;
    - the joint stop as the distance of a joint from the middle between its two neighbours, seg * sin(turn / 2): it
      grows with the turn. (A distance between every second node, 2 * seg * cos(turn / 2), as the radius rope had,
      loses 1 % between 0 and 16 degrees, so a link stretched by 2 % hides a joint at twice its stop.);
    - the clamps and the capsules glide to their new place over the substeps (per update the wrist moves further
      than a link is long);
    - capsules tapered towards the clamps (tapered_radius), and friction of the carrier on the moving capsules;
    - the carrier cannot pass through itself: joints that are not neighbours stay one disc diameter apart.

    `strain` (per chain, after every step): chain length over its nominal length minus one. The links cannot
    stretch, so a strain that stays means the span is pulled taut along the way the carrier lies.
    """

    def __init__(self, lengths: List[float], cfg: CableCarrierConfig):
        disc = cfg.link_chain
        super().__init__(lengths, cfg, segment_length=disc.pitch)
        self.disc = disc
        self._step = disc_native.load().disc_chain_step
        self._params = disc_native.Params(
            substeps=disc.substeps, passes=disc.passes, gravity=disc.gravity,
            # damping per time (cable_carrier.substeps per 0.1 s)
            keep=(1.0 - cfg.damping) ** (cfg.substeps / disc.substeps),
            turn=disc.max_turn, self_distance=disc.disc_diameter, clamp_taper=disc.clamp_taper,
            friction=disc.friction)
        self._start = np.array([c[0] for c in self.chains], dtype=np.intc)
        self._count = np.array([c[1] for c in self.chains], dtype=np.intc)
        self._seg = np.array([c[2] for c in self.chains], dtype=float)
        self._self_skip = np.array([self._skip(seg) for _, _, seg in self.chains], dtype=np.intc)
        self.strain = np.zeros(len(self.chains))
        self._last_pins = self._last_caps = None

    def _skip(self, seg: float) -> int:
        """links along the chain from which two joints must keep one disc diameter: far enough that even the
        tightest bend leaves them further apart than that (nearer ones are held by the joint stops); 0 = none"""
        turn, distance = self.disc.max_turn, self.disc.disc_diameter
        radius = seg / (2.0 * math.sin(turn / 2.0))
        return next((k for k in range(2, 10000) if k * turn >= math.pi
                     or 2.0 * radius * math.sin(k * turn / 2.0) > distance * 1.05), 0) if turn > 0 else 0

    def reset(self, pins, down, caps, settle_steps: int = 60, sides=None, shapes=None):
        self._last_pins = self._last_caps = None
        super().reset(pins, down, caps, settle_steps, sides, shapes)

    def _settle(self, pins, down, caps, steps):
        """settle slowly (LinkChainConfig.settle_keep): the start shape comes to rest without swinging"""
        keep = self._params.keep
        self._params.keep = self.disc.settle_keep
        try:
            super()._settle(pins, down, caps, steps)
        finally:
            self._params.keep = keep

    def step(self, dt, pins, down, caps, motion: Optional[np.ndarray] = None):
        """motion (capsules x 12, optional): per capsule the rotation (3 x 3, row by row) and translation that take
        a point fixed to the capsule's link from its place at the last step to its place now; the friction needs it
        (a housing that turns about its own axis drags the carrier along)"""
        if dt <= 0.0:
            return
        # a fixed substep (max_dt / substeps, 1 ms): the chain's rest shape depends on the substep, so a fixed
        # number of substeps per update made it move with every change of the tick length (the sim's ticks follow
        # sim time: alternating 0.05 / 0.1 s kept it swinging by 2-3 cm at rest, the "shaking discs")
        h = self.cfg.max_dt / max(1, self.disc.substeps)
        self._params.substeps = max(1, int(round(min(dt, self.cfg.max_dt) / h)))
        dt = self._params.substeps * h
        pins_now = np.ascontiguousarray([np.concatenate(p) for p in pins], dtype=float)
        caps_now = shape_records(caps)
        pins_last = pins_now if self._last_pins is None else self._last_pins
        caps_last = caps_now if self._last_caps is None else self._last_caps
        if motion is not None:
            motion = np.ascontiguousarray(motion, dtype=float)
        as_d = lambda arr: arr.ctypes.data_as(ctypes.POINTER(ctypes.c_double))  # noqa: E731
        as_i = lambda arr: arr.ctypes.data_as(ctypes.POINTER(ctypes.c_int))  # noqa: E731
        down = np.ascontiguousarray(down, dtype=float)
        bad = self._step(as_d(self.x), as_d(self.x_prev), as_d(self.inv_mass), len(self.x), as_i(self._start),
                         as_i(self._count), as_d(self._seg), as_i(self._self_skip), len(self.chains),
                         as_d(pins_last), as_d(pins_now), as_d(caps_last), as_d(caps_now),
                         None if motion is None else as_d(motion), len(caps_now), as_d(down),
                         min(dt, self.cfg.max_dt), ctypes.byref(self._params), as_d(self.strain))
        if bad:
            raise FloatingPointError("the carrier link chain is not finite")
        self._last_pins, self._last_caps = pins_now, caps_now

    def tapered_radius(self, caps) -> np.ndarray:
        """Capsule radius per (free node, capsule), as the solver uses it. The clamps sit on the arm, some of them a
        few mm inside the capsule of their own link (widened by the disc radius); the full capsule would push the
        first free node out to its surface: 5 mm over a 1 cm link is a kink of 30 degrees, twice the joint stop. So
        for a node a capsule is no thicker than the clamp's own distance to the capsule axis plus clamp_taper times
        the chain length from that clamp: the chain leaves the capsule at a slope its joints can follow."""
        out = []
        for s0, n, seg in self.chains:
            clamp = self.closest(self.x[[s0 + 1, s0 + n - 2]], caps)[2]
            k = np.arange(2, n - 2)[:, None]
            taper = self.disc.clamp_taper
            limit = np.minimum(clamp[0] + taper * (k - 1) * seg, clamp[1] + taper * (n - 2 - k) * seg)
            out.append(np.minimum(caps[2][None], limit) if taper > 0.0 else np.repeat(caps[2][None], n - 4, axis=0))
        return np.vstack(out)

    def tightened_coarse(self, c: int, caps, iterations: int, segment_length: float) -> np.ndarray:
        """tightened() on every k-th node (k links = about segment_length), clamp nodes kept: the smoothing needs
        the square of the node count in iterations, and only the holds matter for the length"""
        s0, n, seg = self.chains[c]
        k = max(1, int(round(segment_length / seg)))
        idx = [0, 1] + list(range(1 + k, n - 2, k)) + [n - 2, n - 1]
        return self.tighten(self.x[s0:s0 + n][idx].copy(), caps, iterations)

    def twist(self, c: int, ref_start: np.ndarray, ref_end: np.ndarray) -> float:
        """Angle [rad, in (-pi, pi]] by which the end clamp is turned about the carrier axis against the start clamp,
        for the chain's present shape: ref_start (a direction fixed to the start clamp, across the carrier) is carried
        along the chain without twisting (parallel transport) and compared with ref_end at the end clamp."""
        s0, n, _ = self.chains[c]
        return polyline_twist(self.x[s0:s0 + n], ref_start, ref_end)


# ----------------------------------------------------------------------------------------------------------------
# model


class CableModel:
    def __init__(self, urdf_xml: str, cfg: ModelConfig):
        self.cfg = cfg
        self.kin = UrdfKinematics(urdf_xml)
        p = cfg.tf_prefix
        self._base = p + cfg.base_link
        self.arm_joint_names = {j: f"{p}ur5_{j}_joint" for j in ARM_JOINTS}
        missing = [l for l in self._links() if not self.kin.has_link(l)]
        missing += [p + wj.joint for s in cfg.spans for wj in s.joints if p + wj.joint not in self.kin.joints]
        if missing:
            raise ValueError(f"links/joints missing in robot_description: {missing}")
        self._branch: Dict[str, Dict[str, float]] = self._branch_constants()
        if cfg.cable_carrier.enabled:
            try:
                disc_native.load()
            except RuntimeError as exc:
                raise RuntimeError(
                    "cable model: the carrier link chain's solver (native/disc_chain.cpp) is missing or does not "
                    "load, and the carrier link chain is the only chain model (no fallback since 2026-09-30). Build "
                    "the package: catkin "
                    "build mobipick_cable_entanglement (it needs g++). For the geometric check alone set "
                    f"cable_carrier.enabled false. Details: {exc}") from exc
        # the carrier link chains, their lengths and the twist of each span
        self._disc: Optional[CarrierLinkChains] = None
        self._disc_cache: Dict[str, tuple] = {}
        self._twist: Dict[str, float] = {}
        self._twist_zero: Optional[Dict[str, float]] = None
        self._disc_cap_T = None   # link pose of every capsule at the last step of the carrier link chain (friction)
        self._disc_ref = None     # settled carrier link chain at the reference pose (x, x_prev), for _disc_reset_sweep
        self._reset_cache: "OrderedDict[tuple, tuple]" = OrderedDict()   # settled chain per start pose
        self.reset_cache_hits = self.reset_cache_misses = 0
        self.disc_error: Optional[str] = None   # the last error of the carrier link chain (_disc_failed)
        self._entangled_count = 0
        self.level = "ok"
        self._last_q: Optional[Dict[str, float]] = None
        self._tick = 0

    # ---------------------------------------------------------------- frames
    def _links(self) -> List[str]:
        p = self.cfg.tf_prefix
        links = {self._base}
        for s in self.cfg.spans:
            links.add(p + s.start.link)
            links.add(p + s.carrier_anchor.link)
            links.add(p + s.carrier_end_anchor.link)
            if s.carrier_rest_side is not None:
                links.add(p + s.carrier_rest_side.link)
            links.add(p + s.end.link)
        for c in self.cfg.capsules:
            links.add(p + c.link)
        for s in self.cfg.spans:
            for wj in s.joints:
                joint = self.kin.joints.get(p + wj.joint)
                if joint is not None:
                    links.add(joint.parent)
        return sorted(links)

    def _full_q(self, q: Dict[str, float]) -> Dict[str, float]:
        """Accepts joint names with or without prefix / 'ur5_' / '_joint'."""
        out = {}
        for k, v in q.items():
            short = k.split("/")[-1].replace("ur5_", "").replace("_joint", "")
            if short in self.arm_joint_names:
                out[self.arm_joint_names[short]] = float(v)
            else:
                out[k] = float(v)
        return out

    def _frames(self, q: Dict[str, float]):
        poses = self.kin.link_poses(self._links(), q, self._base)
        p = self.cfg.tf_prefix
        anchors, self._carrier_pins, self._carrier_sides, self._carrier_refs = {}, [], [], []
        for s in self.cfg.spans:
            Ts, Te, Tr = poses[p + s.start.link], poses[p + s.end.link], poses[p + s.carrier_anchor.link]
            a = (Ts @ np.r_[s.start.xyz, 1.0])[:3]
            b = (Te @ np.r_[s.end.xyz, 1.0])[:3]
            anchors[s.name] = (a, Ts[:3, :3] @ s.start.direction, b, Te[:3, :3] @ s.end.direction)
            ar = (Tr @ np.r_[s.carrier_anchor.xyz, 1.0])[:3]
            Tre = poses[p + s.carrier_end_anchor.link]
            br = (Tre @ np.r_[s.carrier_end_anchor.xyz, 1.0])[:3]
            self._carrier_pins.append((ar, Tr[:3, :3] @ s.carrier_anchor.direction, br,
                                       Tre[:3, :3] @ s.carrier_end_anchor.direction))
            # a direction across the carrier fixed to each clamp
            self._carrier_refs.append((Tr[:3, :3] @ self._perpendicular(s.carrier_anchor.direction),
                                    Tre[:3, :3] @ self._perpendicular(s.carrier_end_anchor.direction)))
            side = s.carrier_rest_side
            self._carrier_sides.append(None if side is None else poses[p + side.link][:3, :3] @ side.direction)
        caps = []
        self._cap_T = [poses[p + c.link] for c in self.cfg.capsules]
        for c in self.cfg.capsules:
            T = poses[p + c.link]
            ca, cb, sweep, core_radius, box = c.core()
            if box[0] > 0.0:   # the box axes into the frame of the arm base
                box = np.r_[box[:3], (T[:3, :3] @ box[3:].reshape(3, 3)).ravel()]
            caps.append(((T @ np.r_[ca, 1.0])[:3], (T @ np.r_[cb, 1.0])[:3], sweep + self.cfg.cable_carrier_radius,
                         core_radius, box))
        return anchors, caps, poses

    @staticmethod
    def _capsule_arrays(caps):
        return (np.array([c[0] for c in caps]), np.array([c[1] for c in caps]), np.array([c[2] for c in caps]),
                np.array([c[3] for c in caps]), np.array([c[4] for c in caps]))

    @staticmethod
    def _down() -> np.ndarray:
        return np.array([0.0, 0.0, -1.0])  # the arm base frame is level on the mobile base

    def _lead_points(self, span, anchors):
        a, ta, b, tb = anchors[span.name]
        return a + span.lead * ta, b + span.lead * tb

    # ---------------------------------------------------------------- winding (stateless)
    def _joint_geometry(self, wj: WrapJoint, q: Dict[str, float], poses, a: np.ndarray, b: np.ndarray):
        """Cylindrical coordinates of A in the joint's parent-fixed frame and of B in its child frame.

        winding = q_j + theta_B(child) - theta_A(parent) + 2*pi*k: q_j is the absolute (multi-turn) joint angle
        reported by the UR, so a wrist turned by more than half a turn stays wound in the model.
        """
        jname = self.cfg.tf_prefix + wj.joint
        joint = self.kin.joints[jname]
        T_parent = poses[joint.parent] @ joint.origin
        T_child = T_parent @ self.kin.joint_transform_rotation(joint, q.get(jname, 0.0))
        origin, axis = T_parent[:3, 3], T_parent[:3, :3] @ joint.axis
        ref_local = self._perpendicular(joint.axis)
        ra, ta, ha = cylindrical(a, origin, axis, T_parent[:3, :3] @ ref_local)
        rb, tb, hb = cylindrical(b, origin, axis, T_child[:3, :3] @ ref_local)
        sign = 1.0  # the child turns by +q_j about the axis, so its angles grow by q_j in the parent frame
        return ra, ha, rb, hb, sign * q.get(jname, 0.0) + tb - ta

    def _branch_constants(self) -> Dict[str, Dict[str, float]]:
        """2*pi*k per wrap joint such that the winding is in (-pi, pi] with the joint at its neutral angle."""
        out: Dict[str, Dict[str, float]] = {}
        ref = self._full_q(self.cfg.reference)
        for span in self.cfg.spans:
            out[span.name] = {}
            for wj in span.joints:
                qn = dict(ref)
                qn[self.cfg.tf_prefix + wj.joint] = wj.neutral
                anchors, _, poses = self._frames(qn)
                a1, b1 = self._lead_points(span, anchors)
                w = self._joint_geometry(wj, qn, poses, a1, b1)[4]
                out[span.name][wj.joint] = -2.0 * math.pi * round(w / (2.0 * math.pi))
        return out

    @staticmethod
    def _perpendicular(axis_local: np.ndarray) -> np.ndarray:
        v = np.array([1.0, 0.0, 0.0]) if abs(axis_local[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        v = v - (v @ axis_local) * axis_local
        return v / np.linalg.norm(v)

    def _geo(self, span: SpanConfig, q: Dict[str, float], anchors, poses):
        """(L_geo, chord incl. leads, limiting, winding per joint [rad], wrapped per joint)."""
        a1, b1 = self._lead_points(span, anchors)
        chord = float(np.linalg.norm(b1 - a1))
        best, limiting = chord, "chord"
        winding, wrapped = {}, {}
        m = self.cfg.extent_margin
        for wj in span.joints:
            ra, ha, rb, hb, w = self._joint_geometry(wj, q, poses, a1, b1)
            w += self._branch[span.name][wj.joint]
            winding[wj.joint] = w
            lo, hi = wj.extent
            # a housing only holds the cable when both ends are level with it; beyond its end the cable slips over
            inside = lo - m <= ha <= hi + m and lo - m <= hb <= hi + m
            wrapped[wj.joint] = inside
            if not inside:
                continue
            L = cylinder_geodesic(ra, rb, hb - ha, w, wj.radius + self.cfg.cable_carrier_radius)
            if L > best + 1e-9:
                best, limiting = L, wj.joint
        return 2.0 * span.lead + best, 2.0 * span.lead + chord, limiting, winding, wrapped

    # ---------------------------------------------------------------- public API
    def reset(self, q: Dict[str, float], settle_carrier: bool = True, settle_rope: Optional[bool] = None) -> None:
        """Forget the history: the cable hangs freely (untangled) at q, e.g. after a person untangled it.
        settle_rope: the old name of settle_carrier."""
        if settle_rope is not None:
            deprecated_name("CableModel.reset(settle_rope=)", "settle_carrier=")
            settle_carrier = settle_rope
        q = self._full_q(q)
        self._disc = None
        self._disc_cap_T = None
        self._disc_cache, self._twist = {}, {}
        self._tick = 0
        if self.cfg.cable_carrier.enabled and settle_carrier:
            try:
                self._disc_start(q)
            except Exception as exc:  # noqa: BLE001 - the geometric check goes on deciding; the next update retries
                self._disc_failed(exc, q)
        self._entangled_count = 0
        self.level = "ok"
        self._last_q = q

    # ---------------------------------------------------------------- carrier link chain
    def _carrier_link_chains(self) -> CarrierLinkChains:
        return CarrierLinkChains([s.carrier_length for s in self.cfg.spans], self.cfg.cable_carrier)

    def _disc_capsules(self, caps):
        """the capsules widened by the disc radius instead of the carrier radius"""
        a, b, r, core_radius, box = self._capsule_arrays(caps)
        radius = r - self.cfg.cable_carrier_radius + 0.5 * self.cfg.cable_carrier.link_chain.disc_diameter
        return a, b, radius, core_radius, box

    def _chord_twist(self, c: int) -> float:
        """twist of span c for a carrier that runs straight between its clamps (no chain needed: to count whole
        turns)"""
        a, ta, b, tb = self._carrier_pins[c]
        lead = self.cfg.spans[c].lead
        return polyline_twist(np.array([a, a + lead * ta, b + lead * tb, b]), *self._carrier_refs[c])

    def _twist_reference(self) -> Dict[str, tuple]:
        """per span (twist of the settled carrier link chain, twist of the straight carrier) at the reference pose,
        where the
        cable counts as untangled: the zero of the twist"""
        ref = self._full_q(self.cfg.reference)
        _, caps, _ = self._frames(ref)
        chains = self._carrier_link_chains()
        chains.reset(self._carrier_pins, self._down(), self._disc_capsules(caps), sides=self._carrier_sides)
        return {s.name: (chains.twist(c, *self._carrier_refs[c]), self._chord_twist(c))
                for c, s in enumerate(self.cfg.spans)}

    def _reset_key(self, q: Dict[str, float]) -> Optional[tuple]:
        """: the reset cache's key of q (every arm joint rounded to reset_cache_mrad), None when the cache is off"""
        step = self.cfg.cable_carrier.link_chain.reset_cache_mrad * 1e-3
        if step <= 0.0:
            return None
        return tuple(int(round(q.get(self.arm_joint_names[j], 0.0) / step)) for j in ARM_JOINTS)

    def _disc_start(self, q: Dict[str, float]) -> None:
        """the carrier link chains hanging freely at q (reset): started below their chords and settled under gravity;
        with
        LinkChainConfig.reset_cache_mrad a start at a pose seen before reuses that settled chain"""
        if self._twist_zero is None:
            self._twist_zero = self._twist_reference()
        key = self._reset_key(q)
        if key is not None and key in self._reset_cache:
            self._reset_cache.move_to_end(key)
            self._disc_restore(q, self._reset_cache[key])
            self.reset_cache_hits += 1
            return
        self._disc_settle_start(q)
        if key is not None:
            self.reset_cache_misses += 1
            d = self._disc
            self._reset_cache[key] = (dict(q), d.x.copy(), d.x_prev.copy(), d.strain.copy(),
                                      None if d._last_pins is None else d._last_pins.copy(),
                                      None if d._last_caps is None else d._last_caps.copy(), dict(self._twist))
            while len(self._reset_cache) > max(1, self.cfg.cable_carrier.link_chain.reset_cache_size):
                self._reset_cache.popitem(last=False)

    def _disc_restore(self, q: Dict[str, float], snap: tuple) -> None:
        """: the chain as a fresh start at q0 left it, glided from q0 to q (identical to the fresh start when q ==
        q0)"""
        q0, x, x_prev, strain, last_pins, last_caps, twist = snap
        self._disc_cap_T = None
        self._disc_cache = {}
        self._disc = self._carrier_link_chains()
        d = self._disc
        d.x, d.x_prev, d.strain = x.copy(), x_prev.copy(), strain.copy()
        d._last_pins = None if last_pins is None else last_pins.copy()
        d._last_caps = None if last_caps is None else last_caps.copy()
        self._twist = dict(twist)
        joints = [j for j in q0 if j in q]
        if any(abs(q[j] - q0[j]) > 1e-12 for j in joints):
            _, _, _ = self._frames(q0)
            last_T = self._cap_T
            n = max(1, self.cfg.cable_carrier.link_chain.reset_cache_glide_steps)
            for i in range(1, n + 1):
                qi = dict(q)
                qi.update({j: q0[j] + (q[j] - q0[j]) * i / n for j in joints})
                _, caps, _ = self._frames(qi)
                motion = np.array([np.r_[T[:3, :3].ravel(), T[:3, 3]]
                                   for T in (now @ np.linalg.inv(last) for now, last in zip(self._cap_T, last_T))])
                d.step(0.1, self._carrier_pins, self._down(), self._disc_capsules(caps), motion)
                last_T = self._cap_T
            for c, s in enumerate(self.cfg.spans):   # the twist followed along the glide (no whole turn in mrad)
                raw = d.twist(c, *self._carrier_refs[c]) - self._twist_zero[s.name][0]
                prev = self._twist.get(s.name, 0.0)
                self._twist[s.name] = raw + 2.0 * math.pi * round((prev - raw) / (2.0 * math.pi))
        self._frames(q)

    def _disc_settle_start(self, q: Dict[str, float]) -> None:
        """the fresh start of _disc_start (no cache)"""
        _, caps, _ = self._frames(q)
        self._disc_cap_T = None
        self._disc_cache = {}
        self._disc = self._carrier_link_chains()
        dl = self.cfg.cable_carrier.link_chain
        if dl.start_at_reference and dl.reset_sweep_deg > 0.0:
            self._disc_reset_sweep(q, always=True)
            taut = [c for c in range(len(self.cfg.spans)) if self._disc.strain[c] >= dl.strain_tolerance]
            if taut:   # the way from home pulls it taut (e.g. to a place pose): per chain the less taut start
                here = self._carrier_link_chains()
                here.reset(self._carrier_pins, self._down(), self._disc_capsules(caps), sides=self._carrier_sides)
                for c in taut:
                    if here.strain[c] < self._disc.strain[c]:
                        s0, n, _ = here.chains[c]
                        self._disc.x[s0:s0 + n] = here.x[s0:s0 + n]
                        self._disc.x_prev[s0:s0 + n] = here.x_prev[s0:s0 + n]
                        self._disc.strain[c] = here.strain[c]
        else:
            self._disc.reset(self._carrier_pins, self._down(), self._disc_capsules(caps), sides=self._carrier_sides)
            self._disc_reset_sweep(q)
        guess = self._twist_guess(q)
        for c, s in enumerate(self.cfg.spans):
            raw = self._disc.twist(c, *self._carrier_refs[c]) - self._twist_zero[s.name][0]
            self._twist[s.name] = raw + 2.0 * math.pi * round((guess[s.name] - raw) / (2.0 * math.pi))

    def _disc_failed(self, exc: Exception, q: Dict[str, float]) -> None:
        """an error in the carrier link chain must not take the decision down (the monitor skips a tick whose update
        raises, so a failing carrier link chain would stop nothing at all): drop the chain, let the geometric check
        decide
        this tick and start the chain again at the next update (as after an untangle: hanging freely); say so loudly"""
        self._disc, self._disc_cap_T, self._disc_cache = None, None, {}
        self.disc_error = f"{type(exc).__name__}: {exc}"
        message = (f"cable model: the carrier link chain failed ({self.disc_error}); geometric check only for this "
                   "tick, the "
                   "chain starts again (hanging freely) at the next update")
        try:
            import rospy
            rospy.logerr(message)
        except Exception:  # noqa: BLE001 - no ROS
            import warnings
            warnings.warn(message)
        self._frames(q)   # the pins and capsules at q again (the disc code may have left another pose's)

    def _disc_reset_sweep(self, q: Dict[str, float], always: bool = False) -> None:
        """see LinkChainConfig.reset_sweep_deg: a carrier link chain still taut after its start at q is started again at
        the
        reference pose and led to q along the straight joint path; per chain the less taut start is kept. always
        (LinkChainConfig.start_at_reference): the chain always starts that way (self._disc not reset before)"""
        dl = self.cfg.cable_carrier.link_chain
        if not always and (dl.reset_sweep_deg <= 0.0 or float(np.max(self._disc.strain)) < dl.strain_tolerance):
            return
        ref = self._full_q(self.cfg.reference)
        if self._disc_ref is None:   # the settled chain at the reference pose, once
            _, caps, _ = self._frames(ref)
            chains = self._carrier_link_chains()
            chains.reset(self._carrier_pins, self._down(), self._disc_capsules(caps), sides=self._carrier_sides)
            self._disc_ref = (chains.x.copy(), chains.x_prev.copy())
        swept = self._carrier_link_chains()
        swept.x, swept.x_prev = self._disc_ref[0].copy(), self._disc_ref[1].copy()
        _, caps, _ = self._frames(ref)
        swept.step(1e-9, self._carrier_pins, self._down(), self._disc_capsules(caps))   # its last pins and capsules
        last_T = self._cap_T
        joints = [j for j in ref if j in q]
        steps = max(1, int(math.ceil(max((abs(q[j] - ref[j]) for j in joints), default=0.0)
                                     / math.radians(dl.reset_sweep_deg))))
        for i in range(1, steps + dl.reset_settle_steps + 1):
            qi = dict(q)
            qi.update({j: ref[j] + (q[j] - ref[j]) * min(1.0, i / steps) for j in joints})
            _, caps, _ = self._frames(qi)
            motion = np.array([np.r_[T[:3, :3].ravel(), T[:3, 3]]
                               for T in (now @ np.linalg.inv(last) for now, last in zip(self._cap_T, last_T))])
            swept.step(0.1, self._carrier_pins, self._down(), self._disc_capsules(caps), motion)
            last_T = self._cap_T
        self._frames(q)
        for c, (s0, n, _) in enumerate(self._disc.chains):
            if always or swept.strain[c] < self._disc.strain[c]:
                self._disc.x[s0:s0 + n] = swept.x[s0:s0 + n]
                self._disc.x_prev[s0:s0 + n] = swept.x_prev[s0:s0 + n]
                self._disc.strain[c] = swept.strain[c]
        self._disc._last_pins, self._disc._last_caps = swept._last_pins, swept._last_caps

    def _twist_guess(self, q: Dict[str, float], steps: int = 36) -> Dict[str, float]:
        """twist per span at q from the straight carrier, followed from the reference pose along the straight joint
        path so that whole turns are counted (the history before a reset is unknown; the UR joint angles are
        absolute, so this is the untangled way there)"""
        ref = self._full_q(self.cfg.reference)
        out = {s.name: 0.0 for s in self.cfg.spans}
        last = {s.name: self._twist_zero[s.name][1] for s in self.cfg.spans}
        for i in range(1, steps + 1):
            qi = dict(q)
            qi.update({j: ref[j] + (q.get(j, ref[j]) - ref[j]) * i / steps for j in ref})
            self._frames(qi)
            for c, s in enumerate(self.cfg.spans):
                now = self._chord_twist(c)
                out[s.name] += (now - last[s.name] + math.pi) % (2.0 * math.pi) - math.pi
                last[s.name] = now
        self._frames(q)
        return out

    def _disc_state(self, c: int, span: SpanConfig, caps_disc) -> tuple:
        """(L_disc, hooked tool parts, L_disc_topology, twist [rad]) of the carrier link chain of span c"""
        l_disc, hooked, l_topo = self._carrier_length(c, caps_disc, self._disc, coarse=True)
        raw = self._disc.twist(c, *self._carrier_refs[c]) - self._twist_zero[span.name][0]
        prev = self._twist.get(span.name, 0.0)
        self._twist[span.name] = raw + 2.0 * math.pi * round((prev - raw) / (2.0 * math.pi))  # no jump by a turn
        return l_disc, hooked, l_topo, self._twist[span.name]

    def evaluate(self, q: Dict[str, float]) -> CableState:
        """Stateless evaluation (geometry only, no chain history, no hysteresis)."""
        return self.update(q, 0.0, use_carrier=False, hysteresis=False, keep_state=False)

    def update(self, q: Dict[str, float], dt: float, use_carrier: Optional[bool] = None, hysteresis: bool = True,
               with_gradient: bool = True, keep_state: bool = True, use_rope: Optional[bool] = None) -> CableState:
        """use_carrier (default cable_carrier.enabled): with the carrier link chain (history); False = the geometric
        check alone. use_rope: the old name of use_carrier."""
        if use_rope is not None:
            deprecated_name("CableModel.update(use_rope=)", "use_carrier=")
            use_carrier = use_rope
        q = self._full_q(q)
        use_carrier = self.cfg.cable_carrier.enabled if use_carrier is None else use_carrier
        if use_carrier and self._disc is None:
            try:
                self._disc_start(q)   # the first update, or after a failed one: the chain hangs freely at q
            except Exception as exc:  # noqa: BLE001 - the geometric check goes on deciding
                self._disc_failed(exc, q)
        anchors, caps, poses = self._frames(q)
        disc = use_carrier and self._disc is not None
        caps_disc = self._disc_capsules(caps) if disc else None
        refresh = False
        if disc:
            try:
                motion = None
                if self._disc_cap_T is not None:   # how every capsule's link moved since the last step
                    moved = [now @ np.linalg.inv(last) for now, last in zip(self._cap_T, self._disc_cap_T)]
                    motion = np.array([np.r_[T[:3, :3].ravel(), T[:3, 3]] for T in moved])
                self._disc.step(dt, self._carrier_pins, self._down(), caps_disc, motion)
                self._disc_cap_T = self._cap_T
            except Exception as exc:  # noqa: BLE001 - the geometric check goes on deciding
                self._disc_failed(exc, q)
                disc = False
            refresh = self._tick % max(1, self.cfg.cable_carrier.evaluate_every) == 0 or not self._disc_cache
            self._tick += 1
        spans: Dict[str, SpanState] = {}
        torques = {j: 0.0 for j in ARM_JOINTS}
        for c, span in enumerate(self.cfg.spans):
            l_geo, chord, limiting, winding, wrapped = self._geo(span, q, anchors, poses)
            l_disc, disc_hooked, l_disc_topo, twist, strain = None, None, None, None, None
            if disc and refresh:
                try:
                    self._disc_cache[span.name] = self._disc_state(c, span, caps_disc)
                except Exception as exc:  # noqa: BLE001 - the geometric check goes on deciding
                    self._disc_failed(exc, q)
                    disc = False
            if disc:
                l_disc, disc_hooked, l_disc_topo, twist = self._disc_cache[span.name]
                scale = span.free_length / span.carrier_length   # chain lengths in the units of the deciding geometry
                l_disc, l_disc_topo = l_disc * scale, l_disc_topo * scale
                strain = float(self._disc.strain[c])
            l_req = l_geo
            if disc and not self.cfg.decision.geometric_check:   # the carrier link chain alone decides
                l_req, limiting = 0.0, "slack (carrier link chain alone)"
            hooked = []
            if disc_hooked and l_disc > l_req:
                l_req, limiting = l_disc, "carrier link chain (caught on the tool)"
            if disc and self.cfg.cable_carrier.decide_on_topology and l_disc_topo > l_req:
                l_req, limiting = l_disc_topo, "carrier link chain topology"
            if disc_hooked:
                hooked = sorted(disc_hooked)
            # the links cannot stretch: a chain pulled longer than it is needs that much more carrier than it has
            dl = self.cfg.cable_carrier.link_chain
            if (disc and dl.decide_on_strain and strain >= dl.strain_tolerance
                    and span.free_length * (1.0 + strain) > l_req):
                l_req, limiting = span.free_length * (1.0 + strain), "carrier link chain (pulled taut)"
            stretch = l_req / span.free_length - 1.0
            tension = span.stiffness * max(0.0, stretch)
            gradient = self._gradient(span, q) if (with_gradient and tension > 0.0) else {j: 0.0 for j in ARM_JOINTS}
            for j in ARM_JOINTS:
                torques[j] += tension * gradient[j]
            spans[span.name] = SpanState(
                name=span.name, chord=chord, l_geo=l_geo, l_req=l_req,
                free_length=span.free_length,
                stretch=stretch, tension=tension,
                winding_deg={k.replace("ur5_", "").replace("_joint", ""): math.degrees(v) for k, v in winding.items()},
                limiting=limiting.replace("ur5_", "").replace("_joint", ""), hooked_on=hooked, gradient=gradient,
                l_disc=l_disc, l_disc_topology=l_disc_topo, disc_hooked_on=disc_hooked, disc_strain=strain,
                twist_deg=None if twist is None else math.degrees(twist),
                twist_limit_deg=None if twist is None else self._twist_limit_deg(c))
        if keep_state:
            self._last_q = q
        return self._decide(spans, torques, hysteresis)

    def _twist_limit_deg(self, c: int) -> float:
        """twist at which every link of the carrier link chain of span c is at its torsion stop"""
        return (self._disc.chains[c][1] - 1) * self.cfg.cable_carrier.link_chain.twist_per_link_deg

    def _carrier_length(self, c: int, caps_arr, carriers: CarrierLinkChains, coarse: bool = False):
        """(L, hooked tool parts, L_topology) of chain c, from its current (history dependent) shape. carriers: the
        chains to measure; coarse: tighten every k-th node of a fine chain.

        The physical chain is tightened (ChainBase.tightened) and its remaining contacts string-pulled; what stays are
        the points where the cable is really held. L uses only the holds behind the tool (gripper, fingers,
        camera, F/T sensor): the loop caught there. L_topology also keeps the holds around the arm housings;
        the removed radius rope wrapped around them more easily than the stiff triflex cable carrier, which gave false
        alarms on real picks without a stop, so it is logged but does not decide (cable_carrier.decide_on_topology).
        """
        seg = carriers.seg(c)
        if coarse:
            x = carriers.tightened_coarse(c, caps_arr, self.cfg.cable_carrier.tighten_iterations,
                                          self.cfg.cable_carrier.segment_length)
        else:
            x = carriers.tightened(c, caps_arr, self.cfg.cable_carrier.tighten_iterations)
        tol = self.cfg.cable_carrier.contact_tolerance
        _, _, _, pen = ChainBase.closest(x[2:-2], caps_arr)
        nearest = np.argmin(pen, axis=1)
        touching = pen[np.arange(len(pen)), nearest] < tol
        tool = np.array([cap.tool for cap in self.cfg.capsules])

        def pulled_length(mask_caps):
            pts = [x[1]] + [p for p, t, k in zip(x[2:-2], touching, nearest) if t and mask_caps[k]] + [x[-2]]
            sub = tuple(arr[mask_caps] for arr in caps_arr)
            pulled = string_pull(pts, *sub, tol=0.002) if mask_caps.any() else [pts[0], pts[-1]]
            return 2.0 * seg + polyline_length(pulled), pulled

        l_tool, pulled = pulled_length(tool)
        hooked = set()
        if len(pulled) > 2:
            _, _, _, pen_t = ChainBase.closest(np.array(pulled[1:-1]), tuple(arr[tool] for arr in caps_arr))
            names = [cap.name for cap in self.cfg.capsules if cap.tool]
            for row in pen_t:
                k = int(np.argmin(row))
                if row[k] < tol:
                    hooked.add(names[k])
        l_all, _ = pulled_length(np.ones(len(tool), dtype=bool))
        return l_tool, sorted(hooked), l_all

    def _gradient(self, span: SpanConfig, q: Dict[str, float]) -> Dict[str, float]:
        """dL_geo/dq [m/rad] by central differences (the moment arm with which the tension acts on each joint)."""
        eps = 1e-3
        out = {}
        for j in ARM_JOINTS:
            name = self.arm_joint_names[j]
            vals = []
            for sgn in (1.0, -1.0):
                qp = dict(q)
                qp[name] = q.get(name, 0.0) + sgn * eps
                anchors, _, poses = self._frames(qp)
                vals.append(self._geo(span, qp, anchors, poses)[0])
            out[j] = (vals[0] - vals[1]) / (2.0 * eps)
        return out

    def _decide(self, spans: Dict[str, SpanState], torques: Dict[str, float], hysteresis: bool) -> CableState:
        dec = self.cfg.decision
        worst = max(spans.values(), key=lambda s: s.stretch)
        over_torque = [j for j, t in torques.items() if abs(t) >= dec.stop_torque.get(j, float("inf"))]
        predicted = bool(over_torque) or worst.stretch >= dec.stop_stretch
        hooked = sorted({h for s in spans.values() for h in s.hooked_on})
        if predicted:
            reason = (f"{worst.name} span over-stretched by {100 * worst.stretch:.1f}% ({worst.limiting})"
                      + (f", extra torque on {', '.join(over_torque)}" if over_torque else ""))
        elif worst.stretch >= dec.warn_stretch:
            reason = f"{worst.name} span nearly taut ({100 * worst.stretch:+.1f}%, {worst.limiting})"
        else:
            reason = f"slack left ({worst.name} {100 * worst.stretch:+.1f}%)"
        if hooked:
            reason += f"; cable caught on {', '.join(hooked)}"
        if hysteresis:
            self._entangled_count = self._entangled_count + 1 if predicted else 0
            entangled = self._entangled_count >= dec.entangled_ticks
        else:
            entangled = predicted
        if entangled:
            level = "entangled"
        elif worst.stretch >= dec.warn_stretch or hooked:
            level = "warning"
        else:
            level = "ok"
        if hysteresis:
            self.level = level
        return CableState(spans=spans, torques=torques, max_stretch=worst.stretch, worst_span=worst.name,
                          predicted_stop=predicted, level=level, reason=reason)

    # ---------------------------------------------------------------- untangling (sim dashboard button)
    def untangle_plan(self, q: Dict[str, float], joints=("wrist_3", "wrist_2", "wrist_1"),
                      target_stretch: Optional[float] = None, step: float = math.radians(2.0),
                      max_turn: float = math.pi, chain: Optional[dict] = None, speed: float = 0.4):
        """Joint-by-joint motion that gives the cable enough slack again (geometry only, no carrier; with chain, a
        chain_snapshot() of the live model: see _untangle_plan_with_chain).

        Each joint of `joints` in turn gets the angle nearest to its current one (either direction, at most
        max_turn away, within the URDF joint limits) at which the whole cable has target_stretch (default: the
        warning level minus 2 %) or more slack; if no angle reaches that, the one with the least stretch, and the
        next joint continues from there. wrist_3 goes first because it only turns the tool; shoulder and elbow
        never move (large swept volume near tables).
        Returns (steps, final_state, ok): steps = [(joint, target angle [rad]), ...] in execution order.
        """
        if target_stretch is None:
            target_stretch = self.cfg.decision.warn_stretch - 0.02
        if chain is not None:
            return self._untangle_plan_with_chain(q, chain, joints, target_stretch, step, max_turn, speed)
        cur = {j: float(v) for j, v in self._full_q(q).items()}
        steps = []
        state = self.evaluate(cur)
        if state.max_stretch <= target_stretch:
            return steps, state, True
        for j in joints:
            name = self.arm_joint_names[j]
            joint = self.kin.joints[name]
            start = cur[name]
            best_angle, best_state = start, state
            found = False
            for k in range(1, int(max_turn / step) + 1):
                for sgn in (1.0, -1.0):
                    angle = start + sgn * k * step
                    if not joint.lower <= angle <= joint.upper:
                        continue
                    trial = dict(cur)
                    trial[name] = angle
                    st = self.evaluate(trial)
                    if st.max_stretch <= target_stretch:
                        best_angle, best_state, found = angle, st, True
                        break
                    if st.max_stretch < best_state.max_stretch - 1e-4:
                        best_angle, best_state = angle, st
                if found:
                    break
            if abs(best_angle - start) > 1e-6:
                cur[name] = best_angle
                steps.append((j, best_angle))
                state = best_state
            if found:
                return steps, state, True
        return steps, self.evaluate(cur), False

    def _untangle_plan_with_chain(self, q, chain, joints, target_stretch, step, max_turn, speed):
        """untangle_plan from the carried carrier link chain: a stop decided by the chain (pulled taut, caught on the
        tool) can look slack to the geometry alone, and the chain only gets tighter in some directions. Run on a
        planner copy of the live model (same configuration; this model's own chain is replaced by the snapshot), so
        the live chain is never touched. Each joint of `joints` in turn, from the stop pose, is turned in either
        direction in `step` steps up to max_turn, the chain stepped along at `speed` (rad/s, the untangle motion's);
        the nearest angle at which the stretch (the chain, and the geometry while geometric_check is on) reaches
        target_stretch wins. A joint without one leaves the
        pose as it was for the next joint (no partial moves). Returns (steps, final_state, ok): one step when found,
        else no steps, the stop pose's state and ok False ("no slack within max_turn")."""
        cur = {j: float(v) for j, v in self._full_q(q).items()}
        self.chain_restore(chain)
        start = self.update(cur, 0.0, hysteresis=False, with_gradient=False)
        if start.max_stretch <= target_stretch:
            return [], start, True
        dt = step / max(speed, 1e-3)
        best = None   # (k, joint, angle, state)
        for j in joints:
            name = self.arm_joint_names[j]
            joint = self.kin.joints[name]
            for sgn in (1.0, -1.0):
                self.chain_restore(chain)
                # the other direction only searches as far as the slack already found (only a nearer one can win)
                for k in range(1, (int(round(max_turn / step)) if best is None else best[0] - 1) + 1):
                    angle = cur[name] + sgn * k * step
                    if not joint.lower <= angle <= joint.upper:
                        break
                    trial = dict(cur)
                    trial[name] = angle
                    st = self.update(trial, dt, hysteresis=False, with_gradient=False)
                    if st.max_stretch <= target_stretch:
                        best = (k, j, angle, st)
                        break
            if best is not None:
                return [(best[1], best[2])], best[3], True
        return [], start, False

    def chain_snapshot(self) -> Optional[dict]:
        """the carried carrier link chain (copies of its nodes, strain, twist and last capsule poses) for chain_restore
        on a CableModel built from the same configuration; None while there is no chain. The native solver cannot be
        copied, so the restore builds a new one from these arrays (as the reset cache does)."""
        d = self._disc
        if d is None or self._last_q is None:
            return None
        copied = lambda a: None if a is None else a.copy()  # noqa: E731
        return {"q": dict(self._last_q), "x": d.x.copy(), "x_prev": d.x_prev.copy(), "strain": d.strain.copy(),
                "last_pins": copied(d._last_pins), "last_caps": copied(d._last_caps), "twist": dict(self._twist),
                "twist_zero": self._twist_zero,
                "cap_T": None if self._disc_cap_T is None else [T.copy() for T in self._disc_cap_T]}

    def chain_restore(self, snap: dict) -> None:
        """this model's chain becomes the snapshot's (chain_snapshot), the decision history starts afresh"""
        self._disc = self._carrier_link_chains()
        d = self._disc
        d.x, d.x_prev, d.strain = snap["x"].copy(), snap["x_prev"].copy(), snap["strain"].copy()
        d._last_pins = None if snap["last_pins"] is None else snap["last_pins"].copy()
        d._last_caps = None if snap["last_caps"] is None else snap["last_caps"].copy()
        self._twist, self._twist_zero = dict(snap["twist"]), snap["twist_zero"]
        self._disc_cap_T = None if snap["cap_T"] is None else [T.copy() for T in snap["cap_T"]]
        self._disc_cache, self._tick = {}, 0
        self._entangled_count, self.level = 0, "ok"
        self._last_q = dict(snap["q"])
        self._frames(self._last_q)

    # ---------------------------------------------------------------- introspection (markers, logs)
    def disc_points(self) -> Dict[str, np.ndarray]:
        """joints of the carrier link chain per span (; empty without the chain): link k runs from
        point k to point k + 1 and carries its disc at the middle, across the link"""
        if self._disc is None:
            return {}
        return {s.name: self._disc.chain(c).copy() for c, s in enumerate(self.cfg.spans)}

    def anchors(self, q: Dict[str, float]):
        """(anchors per span, capsules) in the arm base frame, for markers."""
        return self._frames(self._full_q(q))[:2]

    def carrier_clamps(self, q: Dict[str, float]) -> Dict[str, tuple]:
        """per span (start, end) where the carrier is really clamped (the carrier's own ends: carrier_start /
        carrier_end), in the
        arm base frame, for markers (the deciding anchors differ at the wrist span and are not drawn)"""
        self._frames(self._full_q(q))
        return {s.name: (pins[0], pins[2]) for s, pins in zip(self.cfg.spans, self._carrier_pins)}

    def snapshot(self) -> dict:
        return {"disc": {k: v.round(4).tolist() for k, v in self.disc_points().items()}}


# the old class names (deprecated; no warning, the config keys and attributes above warn)
RopeConfig = CableCarrierConfig
DiscLinkConfig = LinkChainConfig
DiscChains = CarrierLinkChains
