"""/: when the monitor starts its carrier link chain afresh (chain_reset.ChainResetPolicy): release after rest
(option R)
and reset at the start of an executed trajectory (the shared trajectory initialization), both off by default."""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from mobipick_cable_entanglement.chain_reset import ChainResetPolicy  # noqa: E402

J = ["a", "b"]
REST = {"a": 0.0, "b": 1.0}


def moved(deg):
    return {"a": math.radians(deg), "b": 1.0}


def run(policy, poses, dt=0.1):
    """step through (pose, announce) pairs; the resets with their tick index"""
    out = []
    for i, (q, announce) in enumerate(poses):
        if announce:
            policy.trajectory_started()
        r = policy.step(i * dt, q, J)
        if r is not None:
            out.append((i, r))
    return out


def test_off_by_default_never_resets():
    p = ChainResetPolicy()
    assert not p.enabled
    poses = [(REST, False)] * 200 + [(moved(5), True)] * 10
    assert run(p, poses) == []


def test_release_after_ten_seconds_of_rest_at_the_rest_pose():
    p = ChainResetPolicy(release_after_rest_s=10.0)
    poses = [(REST, False)] * 120 + [(moved(5), False)] * 5   # 11.9 s still, then a motion
    resets = run(p, poses)
    assert len(resets) == 1
    i, r = resets[0]
    assert i == 120 and r.why == "rest" and r.q == REST and abs(r.rest_s - 11.9) < 1e-6


def test_no_release_after_a_shorter_rest_or_within_the_tolerance():
    p = ChainResetPolicy(release_after_rest_s=10.0)
    poses = [(REST, False)] * 50 + [(moved(5), False)] * 5         # 4.9 s rest only
    poses += [(moved(5.3), False)] * 150                          # 0.3 deg drift: still the same rest
    assert run(p, poses) == []


def test_the_rest_counts_from_the_last_motion():
    p = ChainResetPolicy(release_after_rest_s=10.0)
    poses = [(moved(0.2 * i), False) for i in range(150)]          # creeping 0.2 deg per tick: never at rest
    poses += [(moved(40), False)]
    assert run(p, poses) == []


def test_trajectory_start_resets_once_at_the_current_pose_only_when_enabled():
    poses = [(REST, False)] * 5 + [(moved(1), True)] + [(moved(2 + i), False) for i in range(20)]
    assert run(ChainResetPolicy(release_after_rest_s=10.0), poses) == []
    resets = run(ChainResetPolicy(reset_at_trajectory_start=True), poses)
    assert [(i, r.why) for i, r in resets] == [(5, "trajectory")] and resets[0][1].q == moved(1)


def test_an_announced_pose_is_where_the_trajectory_reset_lands_whatever_the_step_sees():
    p = ChainResetPolicy(reset_at_trajectory_start=True)
    p.step(0.0, REST, J)
    p.trajectory_started(0.05, REST)                 # the announcement, at the pose before the swing
    r = p.step(0.6, moved(40), J)                    # the step comes 0.55 s later, mid-swing
    assert (r.t, r.q, r.why, r.announced) == (0.05, REST, "trajectory", True)
    p.trajectory_started()                           # an announcement without a pose: the step's pose as before
    r = p.step(0.7, moved(45), J)
    assert (r.t, r.q, r.announced) == (0.7, moved(45), False)
    assert p.step(0.8, moved(50), J) is None         # each announcement resets once


def test_history_is_kept_within_a_trajectory_and_both_switches_combine():
    p = ChainResetPolicy(release_after_rest_s=10.0, reset_at_trajectory_start=True)
    poses = [(REST, True)] + [(REST, False)] * 119              # trajectory start, then 11.9 s still
    poses += [(moved(5), False)] * 5                             # the next stage of the same trajectory moves
    poses += [(moved(5), True)] + [(moved(10), False)] * 3       # a new trajectory
    resets = run(p, poses)
    assert [(i, r.why) for i, r in resets] == [(0, "trajectory"), (120, "rest"), (125, "trajectory")]


def test_forget_rest_restarts_the_count():
    p = ChainResetPolicy(release_after_rest_s=10.0)
    assert run(p, [(REST, False)] * 90) == []
    p.forget_rest(9.0, REST, J)                                  # e.g. a released stop reset the chain at 9 s
    assert p.step(15.0, moved(5), J) is None                     # 6 s since then: no release


def test_release_during_rest_once_the_arm_has_rested_not_at_the_next_motion():
    p = ChainResetPolicy(release_after_rest_s=10.0, release_during_rest=True)
    poses = [(REST, False)] * 150 + [(moved(5), False)] * 5 + [(moved(5), False)] * 120
    resets = run(p, poses)
    assert [(i, r.why) for i, r in resets] == [(100, "rest"), (250, "rest")]   # 10 s into each rest, once per rest
    assert resets[0][1].q == REST and resets[1][1].q == moved(5)


def test_release_during_rest_with_the_trajectory_reset():
    p = ChainResetPolicy(release_after_rest_s=10.0, reset_at_trajectory_start=True, release_during_rest=True)
    poses = [(REST, True)] + [(REST, False)] * 119 + [(moved(5), True)] + [(moved(10), False)] * 3
    resets = run(p, poses)
    assert [(i, r.why) for i, r in resets] == [(0, "trajectory"), (100, "rest"), (120, "trajectory")]
