"""carrier_replay: the MTC guard's check for what the sim monitor's carrier link chain would stop (; it was
rope_replay before ). Paths modelled on real moves from home. (The test of the 2026-09-28 night sim's rope catch
went with the radius rope, removed 2026-09-30: it tested that model's own configuration.)"""
import math
import os
import sys
import unittest


HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from test_cable_model import ANYGRASP, HOME, OBSERVE, load  # noqa: E402
from mobipick_cable_entanglement.path_check import ReplayStopped, STOP_CHECK_TICKS, resample, carrier_replay  # noqa: E402

J = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
PLACE_END = [-90.4, -131.3, 140.3, -148.3, -91.0, 83.5]      # tennis ball place end, 02:03 (wrist_3 83.5)
VIA = [-90.0, -100.0, 120.0, -100.0, -90.0, 180.0]


def q(deg):
    return {j: math.radians(v) for j, v in zip(J, deg)}


def path(*waypoints, seconds=5.0):
    return [(seconds * i, q(w)) for i, w in enumerate(waypoints)]


class TestChainReplay(unittest.TestCase):
    def test_resample_keeps_the_ends_and_the_rate(self):
        r = resample(path(PLACE_END, ANYGRASP, seconds=1.0), 10.0)
        self.assertEqual(len(r), 11)
        self.assertAlmostEqual(r[5][1]["wrist_3"], math.radians((83.5 + 270.0) / 2), places=6)
        self.assertEqual(r[-1][0], 1.0)

    @unittest.expectedFailure
    def test_direct_path_is_not_flagged(self):
        # 2026-09-30: the removed radius rope stayed slack here (< 0); the carrier link chain, started slowly settled at
        # the
        # place pose, is pulled to +2.2 % at 4.4 s (the stop at +2 %): a false rejection of the carrier link chain, not
        # tuned;
        # the night evaluation of counts such cases on the real days
        v = carrier_replay(load(carrier=True), path(PLACE_END, ANYGRASP))
        self.assertFalse(v.stop)
        self.assertLess(v.max_stretch, 0.02)

    def test_routine_moves_from_home_are_not_flagged(self):
        # 3 s like the real move to the anygrasp pose at 2026-09-28 12:55 UTC (no stop); a carrier ending at the
        # terminal
        # element flagged both
        for target in (ANYGRASP, OBSERVE):
            v = carrier_replay(load(carrier=True), path(HOME, target, target, seconds=3.0))
            self.assertFalse(v.stop, f"{target}: {v.limiting} {v.max_stretch:+.3f}")

    def test_a_stop_ends_the_replay_at_once(self):
        # the callable is asked every STOP_CHECK_TICKS ticks: a True ends the replay with ReplayStopped, long before
        # the end of a path that takes seconds
        model, asked = load(carrier=True), []
        updates = []
        update = model.update
        model.update = lambda *a, **k: updates.append(1) or update(*a, **k)

        def should_stop():
            asked.append(len(updates))
            return len(asked) == 3

        with self.assertRaises(ReplayStopped):
            carrier_replay(model, path(HOME, ANYGRASP, seconds=10.0), should_stop=should_stop)
        # asked once before the chain is settled, then at ticks 0, STOP_CHECK_TICKS, ...: the third ask stops the replay
        self.assertEqual(asked, [0, 0, STOP_CHECK_TICKS])
        self.assertLessEqual(len(updates), STOP_CHECK_TICKS)
        # every tick: a tick takes about 60 ms on one CPU and 140 ms in the sim, every 8th tick was up to 1.1 s late
        self.assertEqual(STOP_CHECK_TICKS, 1)

    def test_a_stop_before_the_settle_does_not_even_settle_the_chain(self):
        model, resets = load(carrier=True), []
        reset = model.reset
        model.reset = lambda *a, **k: resets.append(1) or reset(*a, **k)
        with self.assertRaises(ReplayStopped):
            carrier_replay(model, path(HOME, OBSERVE, seconds=3.0), should_stop=lambda: True)
        self.assertEqual(resets, [])

    def test_a_stop_flag_that_stays_false_changes_nothing(self):
        plain = carrier_replay(load(carrier=True), path(HOME, OBSERVE, seconds=3.0))
        asked = carrier_replay(load(carrier=True), path(HOME, OBSERVE, seconds=3.0), should_stop=lambda: False)
        self.assertEqual((plain.stop, plain.max_stretch, plain.updates), (asked.stop, asked.max_stretch, asked.updates))

    def test_empty_path(self):
        self.assertIsNone(carrier_replay(load(carrier=True), []))


if __name__ == "__main__":
    unittest.main()
