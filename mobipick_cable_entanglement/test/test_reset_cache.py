""": the carrier link chain's reset cache (LinkChainConfig.reset_cache_mrad): a reset at a pose seen before reuses
the chain
settled there instead of settling it again (the MTC guard resets for every path it replays)."""
import math
import os
import sys
from unittest import mock

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from test_cable_model import HOME, OBSERVE, TRANSPORT, q  # noqa: E402
from test_disc_link import fake_rospy, load, loop  # noqa: E402
from mobipick_cable_entanglement.path_check import carrier_replay  # noqa: E402


def _path(start, end):
    return [(0.1 * i, qq) for i, qq in enumerate(loop(start, end))]


def test_off_by_default():
    m = load()
    assert m.cfg.cable_carrier.link_chain.reset_cache_mrad == 0.0
    m.reset(q(TRANSPORT))
    m.reset(q(TRANSPORT))
    assert m.reset_cache_hits == 0 and m.reset_cache_misses == 0 and not m._reset_cache


def test_an_exact_repeat_is_identical_to_a_fresh_reset():
    fresh = load()
    cached = load(reset_cache_mrad=5.0)
    path = _path(TRANSPORT, OBSERVE)
    v_fresh = carrier_replay(fresh, path)
    x_fresh = fresh._disc.x.copy()
    carrier_replay(cached, path)               # miss: settles and stores
    v_cached = carrier_replay(cached, path)    # hit: restored
    assert cached.reset_cache_hits == 1 and cached.reset_cache_misses == 1
    assert (v_cached.stop, v_cached.max_stretch, v_cached.span) == (v_fresh.stop, v_fresh.max_stretch, v_fresh.span)
    assert np.array_equal(cached._disc.x, x_fresh)


def test_a_near_repeat_hits_the_cache_and_stays_close():
    fresh = load()
    cached = load(reset_cache_mrad=5.0)
    cached.reset(q(TRANSPORT))
    near = [v + math.degrees(0.002) for v in TRANSPORT]     # 2 mrad on every joint, same key
    cached.reset(q(near))
    fresh.reset(q(near))
    assert cached.reset_cache_hits == 1
    a = cached.update(q(near), 0.1).spans["wrist"]
    b = fresh.update(q(near), 0.1).spans["wrist"]
    assert abs(a.disc_strain - b.disc_strain) < 0.002
    assert np.abs(cached._disc.x - fresh._disc.x).max() < 0.01


def test_the_cache_keeps_its_size():
    m = load(reset_cache_mrad=5.0, reset_cache_size=2)
    for pose in (HOME, OBSERVE, TRANSPORT):
        m.reset(q(pose))
    assert len(m._reset_cache) == 2 and m.reset_cache_misses == 3
    m.reset(q(HOME))            # evicted: a miss again
    assert m.reset_cache_misses == 4


def test_ros_parameter_switches_it_on():
    with mock.patch.dict(sys.modules, {"rospy": fake_rospy({"/cable_model/reset_cache_mrad": 5.0})}):
        m = load()
    assert m.cfg.cable_carrier.link_chain.reset_cache_mrad == 5.0
