"""best_wrist_3_turn on joint states the MTC cable guard rejected in the sim on 2026-09-28."""
import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from test_cable_model import load  # noqa: E402
from mobipick_cable_entanglement.slack import FLIP_ONLY, best_wrist_3_turn  # noqa: E402

J = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
LIMIT = -0.115
WINDOW = (math.radians(165.7 - 170), math.radians(165.7 + 170))


def q(deg):
    return {j: math.radians(v) for j, v in zip(J, deg)}


class TestBestWrist3Turn(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = load(carrier=False)

    def test_topdown_tennis_ball_gets_slack_by_a_turn(self):
        # topdown_0, rejected at -0.112 (wrist span)
        best = best_wrist_3_turn(self.model, q([-93.8, -79.6, 104.4, -61.4, 44.6, 243.5]), LIMIT, WINDOW)
        self.assertIsNotNone(best)
        turn, stretch = best
        self.assertGreater(turn, 0)          # this pose gets slack with more wrist_3, not less
        self.assertLess(stretch, LIMIT - 0.01)

    def test_flip_makes_this_pose_worse(self):
        # the 180 degree flip alone gives no slack here (+0.24): None for a non-round object
        self.assertIsNone(best_wrist_3_turn(self.model, q([-90.7, -77.4, 100.1, -54.8, 45.1, 241.6]), LIMIT, WINDOW,
                                            FLIP_ONLY))

    def test_flip_helps_a_low_wrist_3_grasp(self):
        # anygrasp_15 at wrist_3 106.2 deg (+0.013): turning by 180 gives slack
        best = best_wrist_3_turn(self.model, q([-112.3, -46.9, 67.9, -69.2, -12.8, 106.2]), LIMIT, WINDOW, FLIP_ONLY)
        self.assertIsNotNone(best)
        self.assertEqual(best[0], 180.0)

    def test_shoulder_span_is_not_fixed_by_wrist_3(self):
        # rejected in the shoulder span (pan -140 deg): no wrist_3 turn helps
        self.assertIsNone(best_wrist_3_turn(self.model, q([-140.4, -24.2, 61.7, -77.7, -48.4, 204.3]), LIMIT, WINDOW))

    def test_turns_stay_in_the_wrist_3_window(self):
        best = best_wrist_3_turn(self.model, q([-93.8, -79.6, 104.4, -61.4, 44.6, 330.0]), LIMIT, WINDOW)
        if best is not None:
            self.assertLessEqual(330.0 + best[0], 165.7 + 170 + 1e-6)

    def test_full_joint_names(self):
        full = {f"mobipick/ur5_{j}_joint": v for j, v in q([-93.8, -79.6, 104.4, -61.4, 44.6, 243.5]).items()}
        self.assertIsNotNone(best_wrist_3_turn(self.model, full, LIMIT, WINDOW))


if __name__ == "__main__":
    unittest.main()
