"""The monitor's chain reset at the start of an executed trajectory lands at the pose of the announcement, not at the
pose of the tick that applies it: a tick 0.6 s late (a busy node) resets at the announced pose and replays the joint
states since, so its chain equals the chain of an on-time tick. A tick 0.5 s into a fast wrist_3 swing used to start
the chain mid-swing, and the rest of the swing over-stretched it past the stop threshold while the guard's replay from
before the swing stayed slack. Stubs instead of a ROS master (needs rospy: the Mobipick image, python3 -m unittest)."""
import collections
import importlib.util
import math
import os
import sys
import threading
import types
import unittest
from unittest import mock

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "src"))
spec = importlib.util.spec_from_file_location(
    "cable_entanglement_monitor", os.path.join(HERE, "..", "nodes", "cable_entanglement_monitor.py"))
monitor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor)

from test_cable_model import OBSERVE  # noqa: E402
from mobipick_cable_entanglement import disc_native  # noqa: E402
from mobipick_cable_entanglement.chain_reset import ChainResetPolicy  # noqa: E402

RATE = 100.0   # joint states (Hz)
SWING = (177.0, 252.0, 1.2)   # wrist_3 from, to (deg) and duration (s): the stopped drill and relay picks


def pose(t, t_swing):
    """OBSERVE with wrist_3 at rest until t_swing, then swinging at constant speed"""
    a, b, dur = SWING
    w3 = a + (b - a) * min(1.0, max(0.0, (t - t_swing) / dur))
    return {j: math.radians(v) for j, v in zip(monitor.ARM_JOINTS, OBSERVE[:5] + [w3])}


class Clock:
    t = 0.0

    def __call__(self):
        return self.t


def node(model, clock):
    m = monitor.CableEntanglementMonitor.__new__(monitor.CableEntanglementMonitor)
    m.model, m.logger, m.mode = model, None, "sim"
    m._lock, m._model_lock = threading.Lock(), threading.RLock()
    m._q, m._q_stamp, m._q_hist = None, None, collections.deque(maxlen=3000)
    m._catch_up_after, m._last_stamp, m._last_update = 0.3, None, None
    m._hold_chain, m._safety, m._armed, m._release_stretch = False, 1, True, None
    m._last_level, m._last_state, m._marker_t = None, None, 1e9
    m._retrigger_margin, m._trajectory_source = 0.005, ""
    m.resets = ChainResetPolicy(reset_at_trajectory_start=True)
    m.pub_state, m.pub_stretch, m.pub_level = mock.Mock(), mock.Mock(), mock.Mock()
    m._poll_safety, m._on_level_change = mock.Mock(), mock.Mock()
    return m


def run(model, tick_times, t_announce=1.0, t_swing=1.0, end=2.4):
    """joint states at RATE from 0 to end, the announcement at t_announce, the monitor ticks at tick_times"""
    clock = Clock()
    with mock.patch.object(monitor.rospy, "get_time", clock), \
            mock.patch.object(monitor.rospy, "get_param", lambda name, default=None: default), \
            mock.patch.object(monitor.rospy, "loginfo") as info, mock.patch.object(monitor.rospy, "loginfo_throttle"):
        m = node(model, clock)
        ticks = sorted(tick_times)
        for k in range(int(round(end * RATE)) + 1):
            t = k / RATE
            clock.t = t
            q = pose(t, t_swing)
            with m._lock:
                m._q, m._q_stamp = q, t
                m._q_hist.append((t, q))
            if abs(t - t_announce) < 1e-9:
                m._on_trajectory_start("mtc solution")
            while ticks and ticks[0] <= t + 1e-9:
                ticks.pop(0)
                m._tick(None)
    return m, [c.args for c in info.call_args_list if "chain starts afresh" in c.args[0]]


def tick_times(skip=()):
    """10 Hz ticks from 0 to 2.4 s without the ones in skip (s)"""
    return [k / 10.0 for k in range(25) if all(abs(k / 10.0 - s) > 1e-9 for s in skip)]


def slack():
    return types.SimpleNamespace(max_stretch=-0.4, level="ok", reason="", predicted_stop=False, to_dict=dict)


class StubModel(unittest.TestCase):
    """the reset and the replay as calls: start the chain at the announced pose, then one update per 0.1 s of the
    joint states since, then the tick's own update"""

    def test_a_late_tick_resets_at_the_announced_pose_and_replays_the_motion_since(self):
        model = mock.Mock()
        model.update.return_value = slack()
        late = tick_times(skip=(1.0, 1.1, 1.2, 1.3, 1.4, 1.5))   # the tick after the announcement comes at 1.6 s
        m, logs = run(model, late)
        model.reset.assert_called_once()
        self.assertAlmostEqual(math.degrees(model.reset.call_args[0][0]["wrist_3"]), SWING[0], places=6)
        names = [c[0] for c in model.method_calls]
        i = names.index("reset")
        after = [c for c in model.method_calls[i + 1:] if c[0] == "update"]
        # the 1.1 to 1.5 s replay and the tick's own update at 1.6 s, each 0.1 s apart
        self.assertEqual([round(c.args[1], 6) for c in after[:6]], [0.1] * 6)
        w3 = [round(math.degrees(c.args[0]["wrist_3"]), 3) for c in after[:6]]
        self.assertEqual(w3, [round(math.degrees(pose(t, 1.0)["wrist_3"]), 3) for t in (1.1, 1.2, 1.3, 1.4, 1.5, 1.6)])
        self.assertEqual(len(logs), 1)
        self.assertIn("reset pose from 0.60 s ago", logs[0][0] % logs[0][1:])

    def test_an_on_time_tick_resets_at_its_own_pose_as_before(self):
        model = mock.Mock()
        model.update.return_value = slack()
        m, logs = run(model, tick_times())
        self.assertAlmostEqual(math.degrees(model.reset.call_args[0][0]["wrist_3"]), SWING[0], places=6)
        self.assertIn("reset pose from 0.00 s ago", logs[0][0] % logs[0][1:])

    def test_a_reset_pose_older_than_the_joint_history_falls_back_to_the_tick_pose(self):
        model = mock.Mock()
        model.update.return_value = slack()
        clock = Clock()
        with mock.patch.object(monitor.rospy, "get_time", clock), \
                mock.patch.object(monitor.rospy, "logwarn") as warn, mock.patch.object(monitor.rospy, "loginfo"):
            m = node(model, clock)
            m._q_hist = collections.deque(maxlen=5)
            for k in range(160):
                clock.t = t = k / RATE
                m._q, m._q_stamp = pose(t, 1.0), t
                m._q_hist.append((t, m._q))
                if k == 100:
                    m._on_trajectory_start("mtc solution")
            m._tick(None)
        warn.assert_called_once()
        self.assertAlmostEqual(math.degrees(model.reset.call_args[0][0]["wrist_3"]),
                               math.degrees(pose(1.59, 1.0)["wrist_3"]), places=6)


@unittest.skipUnless(disc_native.available(), "no carrier link chain solver (g++ or libdisc_chain.so)")
class RealChain(unittest.TestCase):
    """the carrier link chain itself: a late tick gives the chain of an on-time tick"""

    def test_a_late_tick_gives_the_chain_of_an_on_time_tick(self):
        from test_disc_link import load
        on_time, _ = run(load(), tick_times())
        late, logs = run(load(), tick_times(skip=(1.0, 1.1, 1.2, 1.3, 1.4, 1.5)))
        a, b = on_time.model, late.model
        # the on-time tick also steps the fresh chain once at the reset pose: 0.02 mm apart
        self.assertLess(float(np.max(np.abs(a._disc.x - b._disc.x))), 1e-4)
        self.assertAlmostEqual(on_time._last_state.max_stretch, late._last_state.max_stretch, places=3)
        self.assertIn("reset pose from 0.60 s ago", logs[0][0] % logs[0][1:])


if __name__ == "__main__":
    unittest.main()
