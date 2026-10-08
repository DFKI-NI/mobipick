"""Which wrist_3 turn gives the arm cable slack.

A two-finger grasp can be turned about its approach axis without changing what it holds: by 180 degrees for any
object (the Robotiq 2F-140 fingers are symmetric), by any angle for a round object (ball, apple, orange). On the
Mobipick the approach axis (gripper_tcp +x) is the wrist_3 joint axis, so turning the grasp by k turns wrist_3 by k
and leaves the other joints where they were. The cable model then tells which turn gives the wrist span the most
slack; the MTC cable guard still checks the replanned path.
"""
import math
from typing import Dict, Iterable, Optional, Tuple

WRIST_3 = "wrist_3"
FLIP_ONLY = (-180.0, 180.0)
ROUND_TURNS = tuple(float(k) for k in range(-60, 61, 10) if k) + FLIP_ONLY


def _wrist_3_key(q: Dict[str, float]) -> str:
    for key in q:
        if key.split("/")[-1] in (WRIST_3, "ur5_wrist_3_joint"):
            return key
    raise KeyError("no wrist_3 joint in the joint state")


def best_wrist_3_turn(model, q: Dict[str, float], max_stretch: float, wrist_3_limits: Tuple[float, float],
                      turns_deg: Iterable[float] = ROUND_TURNS, margin: float = 0.01
                      ) -> Optional[Tuple[float, float]]:
    """(turn in degrees, predicted max stretch) of the turn that gives the most slack at joint state q (radians),
    or None when no turn keeps wrist_3 within wrist_3_limits (radians) and ends below max_stretch - margin.

    q is typically the guard's worst state of a rejected plan; model is a CableModel (geometry only is used).
    """
    key = _wrist_3_key(q)
    lo, hi = wrist_3_limits
    best = None
    for turn in turns_deg:
        w3 = q[key] + math.radians(turn)
        if not lo <= w3 <= hi:
            continue
        turned = dict(q)
        turned[key] = w3
        stretch = model.evaluate(turned).max_stretch
        if best is None or stretch < best[1]:
            best = (float(turn), stretch)
    if best is None or best[1] >= max_stretch - margin:
        return None
    return best
