""": flat-ended cylinders and rounded boxes as collision shapes next to the capsules. A shape is a core swept by a sphere: a
capsule's core is its axis, a flat cylinder's core is a disc of radius (radius - edge) from a + edge to b - edge."""
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from mobipick_cable_entanglement import disc_native  # noqa: E402
from mobipick_cable_entanglement.cable_model import (  # noqa: E402
    Capsule, ChainBase, CarrierLinkChains, CableCarrierConfig, _shape)
from mobipick_cable_entanglement.geometry import string_pull  # noqa: E402

# a flat cylinder of radius 0.05 m from z 0 to z 0.1 with 5 mm edges, as the model hands it on (no carrier padding)
CYL = Capsule("c", "l", np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.1]), 0.05, edge=0.005)


def shapes(*caps):
    cores = [c.core() for c in caps]
    return tuple(np.array([k[i] for k in cores], dtype=float) for i in range(5))


class TestCore(unittest.TestCase):
    def test_capsule_core_is_its_axis(self):
        c = Capsule("c", "l", np.zeros(3), np.array([0, 0, 0.1]), 0.05)
        a, b, sweep, core, box = c.core()
        self.assertEqual((sweep, core, box[0]), (0.05, 0.0, 0.0))
        np.testing.assert_array_equal(b, [0, 0, 0.1])

    def test_cylinder_core(self):
        a, b, sweep, core, _ = CYL.core()
        np.testing.assert_allclose(a, [0, 0, 0.005])
        np.testing.assert_allclose(b, [0, 0, 0.095])
        self.assertAlmostEqual(sweep, 0.005)
        self.assertAlmostEqual(core, 0.045)


class TestClosest(unittest.TestCase):
    def pen(self, p):
        return ChainBase.closest(np.array([p], float), shapes(CYL))[3][0, 0]

    def test_outside_the_flat_face(self):
        # a capsule of the same radius would reach 5 cm past the face; the flat cylinder ends at z 0.1
        self.assertAlmostEqual(self.pen([0.02, 0.0, 0.13]), 0.03, places=6)

    def test_beside_the_side(self):
        self.assertAlmostEqual(self.pen([0.08, 0.0, 0.05]), 0.03, places=6)

    def test_past_the_rounded_edge(self):
        # diagonal off the edge circle at radius 0.045, z 0.095
        p = np.array([0.045, 0, 0.095]) + 0.01 * np.array([1, 0, 1]) / np.sqrt(2)
        self.assertAlmostEqual(self.pen(p), 0.005, places=6)

    def test_inside_goes_out_through_the_nearest_surface(self):
        caps = shapes(CYL)
        p = np.array([[0.0, 0.03, 0.05], [0.0, 0.0, 0.01]])   # nearer the side / nearer face a
        c, d, dist, pen = ChainBase.closest(p, caps)
        out = c[:, 0] + d[:, 0] / dist[:, 0, None] * caps[2][0]
        np.testing.assert_allclose(out[0], [0, 0.05, 0.05], atol=1e-6)
        np.testing.assert_allclose(out[1], [0, 0, 0.0], atol=1e-6)
        self.assertTrue((pen < 0).all())


class TestStringPull(unittest.TestCase):
    def test_a_string_over_the_flat_face_clears(self):
        # passes 2 cm above the face: a capsule of the cylinder's radius would hold it, the cylinder does not
        pts = [np.array([-0.2, 0, 0.12]), np.array([0, 0, 0.2]), np.array([0.2, 0, 0.12])]
        self.assertEqual(len(string_pull(pts, *shapes(CYL), tol=0.0)), 2)
        capsule = Capsule("c", "l", CYL.a, CYL.b, CYL.radius)
        self.assertEqual(len(string_pull(pts, *shapes(capsule), tol=0.0)), 3)

    def test_a_string_through_the_cylinder_is_held(self):
        pts = [np.array([-0.2, 0, 0.05]), np.array([0, 0.1, 0.05]), np.array([0.2, 0, 0.05])]
        self.assertEqual(len(string_pull(pts, *shapes(CYL), tol=0.0)), 3)


# a box 0.1 x 0.06 x 0.04 m at (0, 0, 0.05), turned by 90 deg about z (its x axis along the link's y), 2 mm edges
BOX = _shape({"name": "b", "link": "l", "box": {"xyz": [0, 0, 0.05], "rpy": [0, 0, 1.5707963267948966],
                                                "size": [0.1, 0.06, 0.04]}, "edge": 0.002})


class TestBox(unittest.TestCase):
    def pen(self, p, shape=BOX):
        return ChainBase.closest(np.array([p], float), shapes(shape))[3][0, 0]

    def test_outside_each_face(self):
        self.assertAlmostEqual(self.pen([0.0, 0.08, 0.05]), 0.03, places=6)    # its x half size 0.05 along y
        self.assertAlmostEqual(self.pen([0.05, 0.0, 0.05]), 0.02, places=6)    # its y half size 0.03 along x
        self.assertAlmostEqual(self.pen([0.0, 0.0, 0.1]), 0.03, places=6)      # its z half size 0.02

    def test_inside_goes_out_through_the_nearest_face(self):
        caps = shapes(BOX)
        c, d, dist, pen = ChainBase.closest(np.array([[0.0, 0.0, 0.065]]), caps)
        out = c[:, 0] + d[:, 0] / dist[:, 0, None] * caps[2][0]
        np.testing.assert_allclose(out[0], [0, 0, 0.07], atol=1e-6)
        self.assertLess(pen[0, 0], 0)

    def test_string_pull_past_the_box(self):
        pts = [np.array([-0.2, 0, 0.05]), np.array([0, 0.2, 0.05]), np.array([0.2, 0, 0.05])]
        self.assertEqual(len(string_pull(pts, *shapes(BOX), tol=0.0)), 3)     # the straight line cuts the box
        pts = [np.array([-0.2, 0, 0.09]), np.array([0, 0, 0.2]), np.array([0.2, 0, 0.09])]
        self.assertEqual(len(string_pull(pts, *shapes(BOX), tol=0.0)), 2)     # 2 cm over its top


@unittest.skipUnless(disc_native.available(), "no carrier link chain solver (g++ or libdisc_chain.so)")
class TestNative(unittest.TestCase):
    def test_cpp_tighten_matches_python_on_all_shape_kinds(self):
        from mobipick_cable_entanglement import disc_native as dn
        caps = shapes(CYL, BOX, Capsule("c", "l", np.array([0.1, 0.0, 0.0]), np.array([0.2, 0.0, 0.1]), 0.03))
        x = np.linspace([-0.2, 0.01, 0.05], [0.3, -0.02, 0.06], 30) + np.random.default_rng(1).normal(0, 0.01, (30, 3))
        fast = ChainBase.tighten(x.copy(), caps, 40)
        available, dn.available = dn.available, (lambda: False)
        try:
            slow = ChainBase.tighten(x.copy(), caps, 40)
        finally:
            dn.available = available
        np.testing.assert_allclose(fast, slow, atol=1e-12)


@unittest.skipUnless(disc_native.available(), "no carrier link chain solver (g++ or libdisc_chain.so)")
class TestDiscChainOnACylinder(unittest.TestCase):
    def test_chain_rests_on_the_cylinder_not_inside(self):
        """a chain hanging over the cylinder settles on its flat top and sides (C++ solver, same core distance)"""
        cfg = CableCarrierConfig()
        chains = CarrierLinkChains([0.4], cfg)
        a, b = np.array([-0.12, 0.0, 0.12]), np.array([0.12, 0.0, 0.12])
        pins = [(a, np.array([0, 0, -1.0]), b, np.array([0, 0, -1.0]))]
        caps = shapes(CYL)
        chains.reset(pins, np.array([0, 0, -1.0]), caps, shapes=[np.linspace(a, b, 41) + [0, 0, 0.0]])
        for _ in range(20):
            chains.step(0.1, pins, np.array([0, 0, -1.0]), caps)
        pen = ChainBase.closest(chains.chain(0)[2:-2], caps)[3][:, 0]
        self.assertGreater(pen.min(), -0.002)


class TestRopeEnd(unittest.TestCase):
    def test_wrist_carrier_ends_in_the_terminal_element_and_the_decision_keeps_its_end(self):
        import yaml
        from mobipick_cable_entanglement.cable_model import CableModel, config_from_dict
        with open(os.path.join(HERE, "..", "config", "cable_model.yaml")) as f:
            d = yaml.safe_load(f)
        urdf = open(os.path.join(HERE, "mobipick_sim_robot_description.urdf")).read()
        m = CableModel(urdf, config_from_dict(d))
        c = [s.name for s in m.cfg.spans].index("wrist")
        anchors, _, poses = m._frames(m._full_q({}))
        T = poses[m.cfg.tf_prefix + "cable_guide_wrist_3_tube_link"]
        local = lambda x: (np.linalg.inv(T) @ np.r_[x, 1.0])[:3]  # noqa: E731
        np.testing.assert_allclose(local(m._carrier_pins[c][2]), [0, 0, 0.027], atol=1e-9)
        np.testing.assert_allclose(local(anchors["wrist"][2]), [0, 0, -0.04], atol=1e-9)


if __name__ == "__main__":
    unittest.main()
