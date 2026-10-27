"""untangle_with_chain: after a stop decided by the carrier link chain (pulled taut), the geometric untangle plan sees
slack or turns the wrong way, and a fresh chain at the stop pose stops again on the way back (sim 2026-10-27: stop at
+27.7 %, "nothing to do", a second protective stop). The chain-aware plan steps a copy of the carried chain along each
candidate turn; the live chain carried along that motion then reaches observe without a stop."""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from test_cable_model import HOME, OBSERVE, q  # noqa: E402
from test_disc_link import load, loop  # noqa: E402

W3 = 5


@pytest.fixture(scope="module")
def stopped():
    """the live model carried from home to observe, then wrist_3 turned down at 0.3 rad/s until it is entangled"""
    m = load()
    m.reset(q(HOME))
    for qq in loop(HOME, OBSERVE):
        m.update(qq, 0.1)
    cur = list(OBSERVE)
    while cur[W3] > 100.0:
        cur[W3] -= math.degrees(0.03)
        state = m.update(q(cur), 0.1)
        if state.level == "entangled":
            return m, cur, state
    pytest.fail("wrist_3 turned down to 100 deg without a stop")


def test_the_stop_comes_from_the_chain_and_the_geometric_plan_turns_further_down(stopped):
    m, stop, state = stopped
    assert "carrier link chain" in state.reason
    assert m.evaluate(q(stop)).max_stretch < 0.0   # geometry alone: slack
    steps, _, _ = m.untangle_plan(q(stop))
    assert [joint for joint, _ in steps] == ["wrist_3"]
    assert math.degrees(steps[0][1]) < stop[W3] - 5.0   # the wrong way: further down


def test_snapshot_restore_continues_the_same_chain(stopped):
    """a restored copy stepped along the same turn as the live model (dt 0.1, the chain moves) stays equal to it; the
    live model gets its snapshot back afterwards, as the fixture is shared"""
    m, stop, _ = stopped
    snap = m.chain_snapshot()
    copy_ = load()
    copy_.chain_restore(snap)
    np.testing.assert_array_equal(copy_._disc.x, m._disc.x)
    try:
        cur = list(stop)
        for _ in range(10):
            cur[W3] += 2.0
            live, restored = m.update(q(cur), 0.1), copy_.update(q(cur), 0.1)
            assert restored.max_stretch == pytest.approx(live.max_stretch, abs=1e-9)
        assert not np.array_equal(m._disc.x, snap["x"])   # the chain did move
        np.testing.assert_allclose(copy_._disc.x, m._disc.x, atol=1e-12)
    finally:
        m.chain_restore(snap)


def test_chain_plan_turns_back_to_slack_without_touching_the_live_chain(stopped):
    m, stop, _ = stopped
    x = m._disc.x.copy()
    planner = load()
    steps, state, ok = planner.untangle_plan(q(stop), chain=m.chain_snapshot(), max_turn=math.radians(90))
    assert ok and len(steps) == 1
    joint, angle = steps[0]
    assert joint == "wrist_3" and stop[W3] < math.degrees(angle) <= stop[W3] + 30.0   # back towards observe
    assert state.max_stretch <= m.cfg.decision.warn_stretch - 0.02
    np.testing.assert_array_equal(m._disc.x, x)


def test_carried_chain_reaches_observe_but_a_fresh_one_stops_again(stopped):
    m, stop, _ = stopped
    snap = m.chain_snapshot()
    steps, _, _ = load().untangle_plan(q(stop), chain=snap, max_turn=math.radians(90))
    slack = list(stop)
    slack[W3] = math.degrees(steps[0][1])
    carried = load()
    carried.chain_restore(snap)
    states = [carried.update(qq, 0.1) for qq in loop(stop, slack, OBSERVE, deg_per_tick=2.0)]
    assert not any(s.level == "entangled" for s in states)
    fresh = load()
    fresh.reset(q(stop))   # today's release: the chain starts afresh at the stop pose
    states = [fresh.update(qq, 0.1) for qq in loop(stop, OBSERVE, deg_per_tick=2.0)]
    assert any(s.level == "entangled" for s in states)


def test_the_search_is_bounded(stopped):
    m, stop, _ = stopped
    steps, state, ok = load().untangle_plan(q(stop), chain=m.chain_snapshot(), max_turn=math.radians(4))
    assert steps == [] and not ok
    assert state.max_stretch > 0.0   # the stop pose's state, for the "no slack within N deg" answer


def test_the_other_direction_stops_at_the_slack_already_found(stopped):
    m, stop, _ = stopped
    planner = load()
    calls = []
    update = planner.update
    planner.update = lambda *args, **kwargs: calls.append(1) or update(*args, **kwargs)
    steps, _, ok = planner.untangle_plan(q(stop), chain=m.chain_snapshot(), max_turn=math.radians(90))
    k = round((math.degrees(steps[0][1]) - stop[W3]) / 2.0)   # 2 deg steps up to the slack
    assert ok and k > 1
    assert len(calls) <= 1 + k + (k - 1)   # the start, k steps up, fewer than k down (not 45)


def test_without_a_chain_there_is_no_snapshot():
    m = load()
    assert m.chain_snapshot() is None
