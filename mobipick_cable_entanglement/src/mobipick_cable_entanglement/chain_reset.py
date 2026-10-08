"""When the monitor starts its carrier link chain afresh (; oscar 2026-10-01), two switches, both off by
default:

release after rest (option R, the interim policy): when the arm has rested at least ``release_after_rest_s`` (no arm
joint more than ``rest_tolerance_deg`` from the pose where the rest began) and starts to move again, the chain starts
afresh at the rest pose: the carrier settles while the arm rests, its history counts only from there. With
``release_during_rest`` the chain starts afresh once the arm HAS rested that long (still resting, once per rest), not at
the next motion: a chain a motion left taut is released during the rest (offline scoring, thinker 2026-10-01 08:30).

reset at trajectory start (the shared trajectory initialization): when an executed trajectory starts (one executed MTC
solution with its approach, grasp and lift; one named move), the chain starts afresh at that pose, exactly as the MTC
cable guard's replay of that trajectory starts (path_check.start_chain); history is kept within the trajectory.

The node and the offline replays share this class, so both run the same rule.
"""
import math
from dataclasses import dataclass
from typing import Dict, Optional, Sequence


@dataclass
class ChainReset:
    t: float                   # time of the reset (s)
    q: Dict[str, float]        # the pose the chain starts afresh at
    why: str                   # "rest" (after rest_s of rest) or "trajectory" (an executed trajectory starts)
    rest_s: float = 0.0        # length of the rest that ended (why "rest")


class ChainResetPolicy:
    def __init__(self, release_after_rest_s: float = 0.0, reset_at_trajectory_start: bool = False,
                 rest_tolerance_deg: float = 0.5, release_during_rest: bool = False):
        self.release_after_rest_s = float(release_after_rest_s)
        self.release_during_rest = bool(release_during_rest)
        self._released = False
        self.reset_at_trajectory_start = bool(reset_at_trajectory_start)
        self.rest_tolerance = math.radians(rest_tolerance_deg)
        self._anchor: Optional[Dict[str, float]] = None
        self._since: Optional[float] = None
        self._last_t: Optional[float] = None
        self._trajectory_started = False

    @property
    def enabled(self) -> bool:
        return self.release_after_rest_s > 0.0 or self.reset_at_trajectory_start

    def trajectory_started(self) -> None:
        """an executed trajectory starts now (the next step() resets when reset_at_trajectory_start)"""
        self._trajectory_started = True

    def step(self, t: float, q: Dict[str, float], joints: Optional[Sequence[str]] = None) -> Optional[ChainReset]:
        """call once per monitor tick before the model update with the current joint positions; returns the reset to
        apply (model.reset(reset.q)) or None"""
        names = list(joints) if joints is not None else sorted(q)
        reset = None
        if self.release_after_rest_s > 0.0:
            if self._anchor is None:
                self._anchor, self._since, self._released = {n: q[n] for n in names}, t, False
            elif max(abs(q[n] - self._anchor[n]) for n in names) > self.rest_tolerance:
                rested = (self._last_t if self._last_t is not None else t) - self._since
                if rested >= self.release_after_rest_s and not self.release_during_rest:
                    reset = ChainReset(t, dict(self._anchor), "rest", rested)
                self._anchor, self._since, self._released = {n: q[n] for n in names}, t, False
            elif self.release_during_rest and not self._released and t - self._since >= self.release_after_rest_s:
                reset = ChainReset(t, dict(self._anchor), "rest", t - self._since)
                self._released = True
        if self._trajectory_started:
            self._trajectory_started = False
            if self.reset_at_trajectory_start:
                reset = ChainReset(t, {n: q[n] for n in names}, "trajectory")
        self._last_t = t
        return reset

    def forget_rest(self, t: float, q: Dict[str, float], joints: Optional[Sequence[str]] = None) -> None:
        """another reset happened (a released stop, the reset service): the rest is counted from here"""
        names = list(joints) if joints is not None else sorted(q)
        self._anchor, self._since, self._released = {n: q[n] for n in names}, t, False
