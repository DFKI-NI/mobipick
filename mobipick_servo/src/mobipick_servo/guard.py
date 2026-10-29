"""Cable guard for MoveIt Servo: pass a commanded arm state on only when the cable model allows it.

Servo streams one joint target per cycle. The camera/FT cable carrier must not be over-stretched, so each target
is
checked with the geometric cable model of mobipick_cable_entanglement, the same call as grasplan's MTC cable guard:
allowed when its largest span stretch stays at or below ``margin`` (the MTC guard's -0.115), or when it is not tighter
than the current state, so the operator can always move back out of a tight pose. A wrist_3 window around the cable
carrier's
neutral angle bounds the winding the model cannot see (as in the MTC guard). Pure logic; the ROS relay is
scripts/cable_guard_relay.py.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, Optional, Tuple

Joints = Dict[str, float]


@dataclass
class Verdict:
    forward: bool
    stretch_now: float
    stretch_cmd: float
    reason: str


def _wrist_3(q: Joints) -> Optional[float]:
    for name, value in q.items():
        if name.split('/')[-1].replace('ur5_', '').replace('_joint', '') == 'wrist_3':
            return value
    return None


def check(
    q_now: Joints,
    q_cmd: Joints,
    max_stretch: Callable[[Joints], float],
    margin: float = -0.115,
    wrist_3_window: Optional[Tuple[float, float]] = None,
    tolerance: float = 1e-4,
) -> Verdict:
    """``max_stretch(q)``: the cable model's largest span stretch [m] for joint state q; ``wrist_3_window`` (lo, hi)
    in rad: a command outside it is refused unless it moves wrist_3 back towards the window."""
    if wrist_3_window is not None:
        lo, hi = wrist_3_window
        now_w3, cmd_w3 = _wrist_3(q_now), _wrist_3(q_cmd)
        if cmd_w3 is not None and not lo <= cmd_w3 <= hi:
            outside_now = 0.0 if now_w3 is None else max(lo - now_w3, now_w3 - hi, 0.0)
            outside_cmd = max(lo - cmd_w3, cmd_w3 - hi)
            if outside_cmd > outside_now + tolerance:
                return Verdict(False, float('nan'), float('nan'), 'wrist_3 would leave the cable window')
    cmd = max_stretch(q_cmd)
    if cmd <= margin:
        return Verdict(True, float('nan'), cmd, 'within the cable margin')  # the usual case: one model call
    now = max_stretch(q_now)
    if cmd <= now + tolerance:
        return Verdict(True, now, cmd, 'tight, but not tighter than now (moving out)')
    return Verdict(False, now, cmd, f'the cable would stretch to {cmd:+.3f} (margin {margin:+.3f}, now {now:+.3f})')
