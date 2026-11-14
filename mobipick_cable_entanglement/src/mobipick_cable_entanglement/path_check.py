"""Would the cable monitor stop the arm along a planned joint path?

The MTC cable guard checks the geometry of every path point (stateless). The sim monitor also runs the carrier link
chain: the carrier with history, which can catch on the tool (camera, F/T sensor) or be pulled taut during a motion,
which geometry alone cannot see. This replays a planned path through the monitor's own decision (the same CableModel
update, chain on, the monitor's rate) from a chain hanging freely at the path's start. That fresh chain forgets what
the motions before the path did to the monitor's chain, so a loop that is already caught or pulled before the path
starts is missed; given the monitor's live chain state (CableModel.chain_state, checked with live_state_problem) the
replay goes on from it instead. A monitor that starts its chain afresh at every trajectory start
(reset_at_trajectory_start) does what the fresh replay does: there the fresh chain is its model. (Until
2026-09-30 a radius rope ran here; it was removed, the carrier link chain is the only chain model; rope_replay is the
old name of carrier_replay.)
"""
import math
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence, Tuple

Sample = Tuple[float, Dict[str, float]]  # (time in s, joint positions in rad)

# a live chain state seeds a replay only when it is this fresh and was taken at the path's first joint positions
LIVE_STATE_MAX_AGE_S = 1.0
LIVE_STATE_Q_TOLERANCE = 0.02   # rad, every arm joint


# a replay asks its should_stop callable every this many ticks (one tick is a CableModel.update, 60 ms offline and about
# 140 ms in the sim; the check is a flag read, so every tick)
STOP_CHECK_TICKS = 1


class ReplayStopped(Exception):
    """carrier_replay was asked to stop (its should_stop returned True) before it reached the end of the path."""


@dataclass
class CarrierVerdict:
    stop: bool                 # the monitor would predict a protective stop somewhere on the path
    max_stretch: float         # largest stretch along the path (carrier link chain included)
    span: str
    # what set it ("carrier link chain (pulled taut)", "carrier link chain (caught on the tool)", a joint, "chord")
    limiting: str
    q: Dict[str, float]        # joint positions (rad) at the worst point
    t: float                   # time along the path of the worst point
    updates: int


def resample(samples: Sequence[Sample], rate: float) -> List[Sample]:
    """Linear interpolation of a joint path at a fixed rate (the monitor's tick), ending exactly at the last point."""
    if not samples:
        return []
    out = [samples[0]]
    start, end = samples[0][0], samples[-1][0]
    i, k = 0, 1
    while start + k / rate < end - 1e-9:   # times by index: no accumulated rounding
        t = start + k / rate
        while samples[i + 1][0] < t:
            i += 1
        (t0, q0), (t1, q1) = samples[i], samples[i + 1]
        a = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
        out.append((t, {j: q0[j] + a * (q1[j] - q0[j]) for j in q0}))
        k += 1
    if len(samples) > 1:
        out.append(samples[-1])
    return out


def start_chain(model, q: Dict[str, float]) -> None:
    """The settled chain an executed trajectory starts from: the guard's replay of a planned trajectory and the
    monitor's reset at the start of its execution (chain_reset, the shared trajectory initialization) both start here,
    with the same model config, shapes and settle."""
    model.reset(q)


def _arm_joint(name: str) -> str:
    return name.split("/")[-1].replace("ur5_", "").replace("_joint", "")


def live_state_problem(model, state: Optional[dict], q0: Dict[str, float], now: float,
                       max_age: float = LIVE_STATE_MAX_AGE_S,
                       q_tolerance: float = LIVE_STATE_Q_TOLERANCE) -> Optional[str]:
    """Why the monitor's chain state (CableModel.chain_state plus the monitor's "stamp", its time in s, and
    "reset_at_trajectory_start") cannot be the start of a replay of a path that starts at q0 at time now, or None when
    it can: none, a monitor that starts its chain afresh at the trajectory start anyway, another model config, stale,
    or taken at other joint positions (the arm moved since, or the path does not start where the arm is)."""
    if not state:
        return "no chain state"
    if state.get("reset_at_trajectory_start"):
        return "the monitor starts its chain afresh when the trajectory starts, as the fresh replay does"
    if state.get("fingerprint") != model.fingerprint():
        return "the monitor's cable model differs from the guard's (config or robot description)"
    stamp = state.get("stamp")
    if stamp is None or now - float(stamp) > max_age:
        return "stale" if stamp is None else f"stale ({now - float(stamp):.1f} s old)"
    live = {_arm_joint(j): v for j, v in state.get("q", {}).items()}
    start = {_arm_joint(j): v for j, v in q0.items()}
    common = [j for j in start if j in live]
    if not common:
        return "no common joints"
    worst = max(common, key=lambda j: abs(live[j] - start[j]))
    if abs(live[worst] - start[worst]) > q_tolerance:
        return f"the arm is elsewhere ({worst} {math.degrees(live[worst] - start[worst]):+.1f} deg from the path start)"
    return None


def carrier_replay(model, samples: Sequence[Sample], rate: float = 10.0,
                   start_state: Optional[dict] = None,
                   should_stop: Optional[Callable[[], bool]] = None) -> Optional[CarrierVerdict]:
    """Replay samples (time-ordered) through model.update with the carrier link chain on, from a chain hanging freely at
    the first sample, or with start_state (a CableModel.chain_state of the same config, see live_state_problem) from
    that chain.

    model is a CableModel whose config has the chain enabled (cable_carrier.enabled) (it is reset here, so use one not
    shared with a live
    monitor). None for an empty path.

    should_stop: called before the chain is settled and then every STOP_CHECK_TICKS ticks; when it returns True the replay
    ends at once with ReplayStopped (a long path takes seconds, and a caller whose goal was cancelled must not wait for
    the verdict). The settling of the start chain (about 60 model steps) itself is not interrupted.
    """
    path = resample(list(samples), rate)
    if not path:
        return None
    if should_stop is not None and should_stop():
        raise ReplayStopped("stopped before the chain was settled")
    if start_state is None:
        start_chain(model, path[0][1])
    else:
        model.restore_chain_state(start_state)
    worst, stop, last_t = None, False, path[0][0]
    for tick, (t, q) in enumerate(path):
        if should_stop is not None and tick % STOP_CHECK_TICKS == 0 and should_stop():
            raise ReplayStopped(f"stopped at tick {tick} of {len(path)}")
        state = model.update(q, max(0.0, t - last_t), use_carrier=True, hysteresis=False, with_gradient=True)
        last_t = t
        stop = stop or bool(state.predicted_stop)
        if worst is None or state.max_stretch > worst[0].max_stretch:
            worst = (state, t, q)
    state, t, q = worst
    return CarrierVerdict(stop=stop, max_stretch=state.max_stretch, span=state.worst_span,
                       limiting=state.spans[state.worst_span].limiting, q=dict(q), t=t - path[0][0],
                       updates=len(path))


# the old names (deprecated)
RopeVerdict = CarrierVerdict


def rope_replay(model, samples: Sequence[Sample], rate: float = 10.0) -> Optional[CarrierVerdict]:
    """the old name of carrier_replay"""
    from .cable_model import deprecated_name
    deprecated_name("path_check.rope_replay", "path_check.carrier_replay")
    return carrier_replay(model, samples, rate)
