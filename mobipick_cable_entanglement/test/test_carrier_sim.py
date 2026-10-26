""" step 3: the Gazebo cable carrier's wiring (carrier_sim.py): the clamps, shapes and the initial shape on the
links Gazebo keeps after merging the fixed ones, and the plugin SDF."""
import os
import sys
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "src"))

from test_cable_model import HOME, OBSERVE, q  # noqa: E402
from mobipick_cable_entanglement import carrier_sim as cs  # noqa: E402
from mobipick_cable_entanglement.cable_model import CableModel, config_from_dict  # noqa: E402
from mobipick_cable_entanglement.kinematics import UrdfKinematics, make_transform, rpy_to_matrix  # noqa: E402

P = "mobipick/"
with open(os.path.join(HERE, "mobipick_sim_robot_description.urdf")) as f:
    URDF = f.read()
with open(os.path.join(HERE, "..", "config", "cable_model.yaml")) as f:
    CONFIG = yaml.safe_load(f)


def parse(sdf):
    return ET.fromstring(sdf.split("?>", 1)[1])


def pose_matrix(text):
    v = [float(x) for x in text.split()]
    return make_transform(rpy_to_matrix(*v[3:]), v[:3])


class Wiring(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.kin = UrdfKinematics(URDF)
        cls.model = CableModel(URDF, config_from_dict(CONFIG, tf_prefix="mobipick"))

    def test_rpy_round_trip(self):
        for rpy in [(0.1, -0.4, 2.0), (3.1, 0.2, -1.2), (0, 0, 0), (-2.5, 1.2, 0.7)]:
            back = cs.matrix_to_rpy(rpy_to_matrix(*rpy))
            np.testing.assert_allclose(rpy_to_matrix(*back), rpy_to_matrix(*rpy), atol=1e-9)

    def test_fixed_links_merge_into_their_moving_ancestor(self):
        link, T = cs.moving_ancestor(self.kin, P + "cable_guide_forearm_tube_link")
        self.assertEqual(link, P + "ur5_forearm_link")
        poses = self.kin.link_poses([link, P + "cable_guide_forearm_tube_link"], q(OBSERVE))
        np.testing.assert_allclose(np.linalg.inv(poses[link]) @ poses[P + "cable_guide_forearm_tube_link"], T, atol=1e-9)
        self.assertEqual(cs.moving_ancestor(self.kin, P + "cable_guide_wrist_3_tube_link")[0], P + "ur5_wrist_3_link")
        link, T = cs.moving_ancestor(self.kin, P + "ur5_wrist_3_link")   # a moving link is its own
        self.assertEqual(link, P + "ur5_wrist_3_link")
        np.testing.assert_allclose(T, np.eye(4))

    def test_shapes_keep_their_place_on_the_gazebo_link(self):
        shapes = {s["name"]: s for s in cs.shapes_in_gazebo(self.kin, P, CONFIG["capsules"])}
        for c in CONFIG["capsules"]:
            link, T_link = cs.moving_ancestor(self.kin, P + c["link"])
            if "box" in c:
                s = shapes[c["name"]]
                self.assertEqual((s["link"], s["type"]), (link, "box"))
                np.testing.assert_allclose(pose_matrix(s["pose"]),
                                           T_link @ make_transform(rpy_to_matrix(*c["box"]["rpy"]), c["box"]["xyz"]),
                                           atol=1e-5)
                continue
            s = shapes[c["name"]]
            self.assertEqual((s["link"], s["type"]), (link, "cylinder"))
            r, length = (float(v) for v in s["size"].split())
            T = pose_matrix(s["pose"])
            ends = [T @ np.array([0, 0, z, 1.0]) for z in (-length / 2, length / 2)]
            a, b = (T_link @ np.append(c[k], 1.0) for k in ("a", "b"))
            self.assertAlmostEqual(r, c["radius"], places=5)
            np.testing.assert_allclose(sorted([tuple(e[:3].round(4)) for e in ends]),
                                       sorted([tuple(a[:3].round(4)), tuple(b[:3].round(4))]), atol=2e-4)

    def test_round_capsules_get_spheres(self):
        shapes = cs.shapes_in_gazebo(self.kin, P, [{"name": "c", "link": "ur5_wrist_1_link", "a": [0, 0, 0],
                                                    "b": [0, 0, 0.1], "radius": 0.02}])
        self.assertEqual([s["type"] for s in shapes], ["cylinder", "sphere", "sphere"])

    def test_initial_shape_runs_from_clamp_to_clamp(self):
        span = CONFIG["spans"]["wrist"]
        for pose in (HOME, OBSERVE):
            pts = cs.initial_points_from_model(self.model, q(pose), CONFIG, P)
            parent, pa = cs.anchor_in_gazebo(self.kin, P, span["carrier_start"])
            child, pb = cs.anchor_in_gazebo(self.kin, P, span["carrier_end"])
            poses = self.kin.link_poses([parent, child], self.model._full_q(q(pose)))
            pb_parent = (np.linalg.inv(poses[parent]) @ poses[child] @ np.append(pb, 1.0))[:3]
            self.assertLess(np.linalg.norm(pts[0] - pa), 0.02)     # the plugin uses it within 2 cm of each clamp
            self.assertLess(np.linalg.norm(pts[-1] - pb_parent), 0.02)
            length = np.linalg.norm(np.diff(pts, axis=0), axis=1).sum()
            self.assertAlmostEqual(length, span["carrier_free_length"], delta=0.02)

    def test_initial_shape_keeps_every_joint_within_the_plugins_bend_limit(self):
        # the plugin resamples the points to equal links; the model's settled shape bent single joints to 0.61 rad at
        # home against the limit link / bend radius = 0.51 rad, so the chain started inside its joint limits
        length = CONFIG["spans"]["wrist"]["carrier_free_length"]
        links = cs.chain_links(length)
        limit = length / links / cs.bend_radius_from_config(CONFIG)
        for pose in (HOME, OBSERVE):
            pts = cs.initial_points_from_model(self.model, q(pose), CONFIG, P)
            seen = cs.joint_bends(cs.resample_equal(pts, links))
            self.assertLessEqual(seen.max(), limit, pose)

    def test_limit_bends_keeps_ends_and_length(self):
        corner = np.array([[0, 0, 0], [0.2, 0, 0], [0.2, 0.002, 0], [0.4, 0.002, 0]], dtype=float)
        shape = cs.limit_bends(corner, 20, 0.3)
        self.assertEqual(len(shape), 21)
        np.testing.assert_allclose(shape[[0, -1]], corner[[0, -1]], atol=1e-9)
        self.assertLessEqual(cs.joint_bends(shape).max(), 0.3)
        self.assertAlmostEqual(np.linalg.norm(np.diff(shape, axis=0), axis=1).sum(), 0.4, delta=0.01)

    def test_plugin_sdf(self):
        pts = cs.initial_points_from_model(self.model, q(HOME), CONFIG, P)
        root = parse(cs.plugin_sdf(self.kin, CONFIG, P, "mobipick", "mobipick", initial_points=pts,
                                   stop_service="/mobipick/rqt_sim_gui/set_protective_stop", stop_strain=0.02,
                                   stop_force=30.0))
        plugin = root.find("model/plugin")
        self.assertEqual(plugin.get("filename"), "libgazebo_jointed_chain.so")
        self.assertEqual(root.find("model/static").text, "true")
        val = {e.tag: e.text for e in plugin if e.tag != "shape"}
        self.assertEqual((val["parent_link"], val["child_link"], val["host_model"]),
                         (P + "ur5_forearm_link", P + "ur5_wrist_3_link", "mobipick"))
        self.assertEqual(float(val["length"]), CONFIG["spans"]["wrist"]["carrier_free_length"])
        self.assertAlmostEqual(float(val["mass_per_link"]) * int(val["links"]), 0.35 * 0.52, places=4)
        self.assertEqual(float(val["bending_stiffness"]), 0.03)   # PLACEHOLDER until 
        # the model's carrier: 3.5 cm bend radius from the disc sizes, links of at most 1.8 cm (29 for 0.52 m)
        self.assertAlmostEqual(float(val["bend_radius"]), 0.035, delta=0.0005)
        self.assertEqual(int(val["links"]), 29)
        self.assertEqual(len(val["initial_points"].split()), 3 * len(pts))
        self.assertEqual(val["stop_service"], "/mobipick/rqt_sim_gui/set_protective_stop")
        self.assertEqual(len(plugin.findall("shape")), len(CONFIG["capsules"]))
        # the published frame is the arm base, merged into the mobile base's root link
        ref, T = cs.moving_ancestor(self.kin, P + "ur5_base_link")
        self.assertEqual(val["reference_link"], ref)
        np.testing.assert_allclose(pose_matrix(val["reference_offset"]), T, atol=1e-5)

    def test_no_stop_call_unless_asked(self):
        plugin = parse(cs.plugin_sdf(self.kin, CONFIG, P, "mobipick", "mobipick")).find("model/plugin")
        self.assertIsNone(plugin.find("stop_service"))
        self.assertIsNone(plugin.find("initial_points"))

    def test_overrides(self):
        plugin = parse(cs.plugin_sdf(self.kin, CONFIG, P, "mobipick", "mobipick",
                                     chain={"bending_stiffness": 0.0, "links": 20})).find("model/plugin")
        self.assertEqual((plugin.find("bending_stiffness").text, plugin.find("links").text), ("0.0", "20"))
        plugin = parse(cs.plugin_sdf(self.kin, CONFIG, P, "mobipick", "mobipick",
                                     chain={"bend_radius": 0.1})).find("model/plugin")
        self.assertEqual(float(plugin.find("bend_radius").text), 0.1)


if __name__ == "__main__":
    unittest.main()
