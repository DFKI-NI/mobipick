"""The monitor's carrier discs keep the carrier colour while a span is ok and take the level colour when it warns or
entangles (the markers are also what the sim draws of the cable in Gazebo)."""
import importlib.util
import os
import sys
import types
import unittest
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
spec = importlib.util.spec_from_file_location(
    "cable_entanglement_monitor", os.path.join(HERE, "..", "nodes", "cable_entanglement_monitor.py"))
monitor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor)


def stub():
    m = monitor.CableEntanglementMonitor.__new__(monitor.CableEntanglementMonitor)
    m.frame = "robot/arm_base"
    m.cfg = types.SimpleNamespace(cable_carrier=types.SimpleNamespace(
        link_chain=types.SimpleNamespace(disc_diameter=0.035, disc_height=0.005)))
    return m


JOINTS = [(0.0, 0.0, 0.0), (0.0, 0.0, 0.1), (0.0, 0.1, 0.1)]


class DiscColours(unittest.TestCase):
    def colours(self, *level):
        markers = stub()._disc_markers(0, JOINTS, None, *level)
        discs = [m for m in markers if m.ns == "cable_discs"]
        self.assertEqual(len(discs), len(JOINTS) - 1)
        return {(round(m.color.r, 3), round(m.color.g, 3), round(m.color.b, 3)) for m in discs}

    def test_ok_keeps_the_carrier_colour(self):
        self.assertEqual(self.colours(), {monitor.CARRIER_COLOR})
        self.assertEqual(self.colours("ok"), {monitor.CARRIER_COLOR})

    def test_warning_and_entangled_take_the_level_colour(self):
        for level in ("warning", "entangled"):
            self.assertEqual(self.colours(level), {monitor.LEVEL_COLORS[level]})

    def test_the_line_stays_dark(self):
        line = [m for m in stub()._disc_markers(0, JOINTS, None, "entangled") if m.ns == "cable_links"]
        self.assertEqual(len(line), 1)
        self.assertLess(line[0].color.r, 0.2)


class Pub:
    def __init__(self, listeners):
        self.listeners, self.sent = listeners, []

    def get_num_connections(self):
        return self.listeners

    def publish(self, arr):
        self.sent.append(arr)


class DiscTopic(unittest.TestCase):
    """~disc_markers carries only the discs, and nothing is built while nobody listens"""

    def setUp(self):
        patcher = mock.patch.object(monitor.rospy.Time, "now", lambda: monitor.rospy.Time(0))
        patcher.start()
        self.addCleanup(patcher.stop)

    def monitor(self, rviz, gazebo):
        m = stub()
        m.pub_markers, m.pub_disc_markers = Pub(rviz), Pub(gazebo)
        m._markers_cleared, m.show_capsules = True, True
        m.model = types.SimpleNamespace(disc_points=lambda: {"wrist": JOINTS})
        m.cfg.decision = types.SimpleNamespace(stop_stretch=0.02, warn_stretch=-0.05)
        m.cfg.cable_carrier.link_chain.strain_tolerance = 0.05
        m._capsule_markers = lambda q, stamp: [monitor.Marker(ns="collision_capsules")]
        return m

    def state(self, stretch):
        span = types.SimpleNamespace(stretch=stretch, hooked_on=[], disc_strain=None)
        return types.SimpleNamespace(spans={"wrist": span})

    def test_nobody_listens_nothing_built(self):
        m = self.monitor(0, 0)
        m.model = None   # would raise if the markers were built
        m._publish_markers({}, self.state(0.0))
        self.assertEqual((m.pub_markers.sent, m.pub_disc_markers.sent), ([], []))

    def test_only_the_drawing_listens_discs_only(self):
        m = self.monitor(0, 1)
        m._publish_markers({}, self.state(0.03))
        self.assertEqual(m.pub_markers.sent, [])
        (arr,) = m.pub_disc_markers.sent
        self.assertEqual({x.ns for x in arr.markers}, {"cable_discs"})
        self.assertEqual({(x.color.r, x.color.g, x.color.b) for x in arr.markers}, {monitor.LEVEL_COLORS["entangled"]})

    def test_rviz_gets_everything(self):
        m = self.monitor(1, 0)
        m._publish_markers({}, self.state(0.0))
        (arr,) = m.pub_markers.sent
        self.assertEqual({x.ns for x in arr.markers}, {"cable_discs", "cable_links", "cable_text", "collision_capsules"})
        self.assertEqual(m.pub_disc_markers.sent, [])


if __name__ == "__main__":
    unittest.main()
