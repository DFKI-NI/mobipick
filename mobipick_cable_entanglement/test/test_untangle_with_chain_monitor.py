"""untangle_with_chain in the monitor node: the plan comes from the planner copy with a snapshot of the live chain
and names that model; the "no reset" after the untangle and at the release holds only when a chain-aware untangle
moved the arm during that stop; a rest release of the chain waits while the stop locks the arm. Stubs instead of a
ROS master, the controller manager and the trajectory action (needs rospy: the Mobipick image, python3 -m unittest)."""
import importlib.util
import math
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

Q = {j: 0.1 * i for i, j in enumerate(monitor.ARM_JOINTS)}
STOP = 3   # protective stop (STOPPED_SAFETY)
NORMAL = 1


def state(stretch):
    return types.SimpleNamespace(max_stretch=stretch, level="entangled" if stretch > 0.02 else "ok")


def stub(flag):
    m = monitor.CableEntanglementMonitor.__new__(monitor.CableEntanglementMonitor)
    m.params = {"~untangle_with_chain": flag}
    m._cable_param = lambda name, default: m.params.get("~" + name, default)
    m.cfg = types.SimpleNamespace(decision=types.SimpleNamespace(stop_stretch=0.02, geometric_check=True))
    m.model = mock.Mock()
    m.model.chain_snapshot.return_value = {"chain": "snapshot"}
    m.model.evaluate.return_value = state(-0.107)
    m._planner = mock.Mock()
    m._urdf = "<robot/>"
    m._lock, m._model_lock, m._planner_lock = mock.MagicMock(), mock.MagicMock(), mock.MagicMock()
    m._q = dict(Q)
    m._safety, m.logger, m._last_state = NORMAL, None, state(0.277)
    m._chain_untangled = m._hold_chain = m._stopped_by_us = False
    m.mode = "sim"
    m._release_stretch = None
    m.resets = mock.Mock()
    return m


class Plan(unittest.TestCase):
    def setUp(self):
        patcher = mock.patch.object(monitor.rospy, "get_param", lambda name, default=None: default)
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_off_plans_with_the_geometry_as_before(self):
        m = stub(False)
        m.model.untangle_plan.return_value = ([], state(-0.107), True)
        steps, _, ok, text, chain = m._untangle_plan(Q)
        m.model.untangle_plan.assert_called_once_with(Q)
        m._planner.untangle_plan.assert_not_called()
        self.assertFalse(chain)
        self.assertNotIn("carrier link chain", text)

    def test_on_plans_on_the_copy_from_the_live_chain_and_names_the_model(self):
        m = stub(True)
        m._planner.untangle_plan.return_value = ([("wrist_3", math.radians(232.6))], state(-0.063), True)
        steps, _, ok, text, chain = m._untangle_plan(Q)
        kwargs = m._planner.untangle_plan.call_args[1]
        self.assertEqual(kwargs["chain"], {"chain": "snapshot"})
        self.assertAlmostEqual(kwargs["max_turn"], math.radians(90.0))
        m.model.untangle_plan.assert_not_called()
        self.assertTrue(ok and chain)
        self.assertIn("cable stretch (carrier link chain and geometry) +27.7% -> -6.3%", text)

    def test_no_slack_within_the_bound_says_so(self):
        m = stub(True)
        m._planner.untangle_plan.return_value = ([], state(0.277), False)
        _, _, ok, text, _ = m._untangle_plan(Q)
        self.assertFalse(ok)
        self.assertIn("no slack within 90 deg of wrist_3, wrist_2, wrist_1", text)

    def test_without_the_geometric_check_the_text_names_the_chain_alone(self):
        m = stub(True)
        m.cfg.decision.geometric_check = False
        m._planner.untangle_plan.return_value = ([("wrist_3", math.radians(232.6))], state(-0.063), True)
        text = m._untangle_plan(Q)[3]
        self.assertIn("cable stretch (carrier link chain) +27.7%", text)

    def test_the_copy_is_built_from_the_same_description_and_configuration(self):
        m = stub(True)
        m._planner = None
        with mock.patch.object(monitor, "CableModel") as model:
            model.return_value.untangle_plan.return_value = ([], state(0.277), False)
            m._untangle_plan(Q)
        model.assert_called_once_with(m._urdf, m.cfg)


class Release(unittest.TestCase):
    def setUp(self):
        patcher = mock.patch.object(monitor.rospy, "get_time", lambda: 100.0)
        patcher.start()
        self.addCleanup(patcher.stop)

    def stop_and_release(self, m, chain_untangled):
        m._set_safety(STOP, "test")
        if chain_untangled:
            m._chain_untangled = True   # what _untangle sets after a chain-aware motion
            m._last_state = state(-0.063)
        m._set_safety(NORMAL, "test")

    def test_release_after_a_chain_untangle_keeps_the_chain(self):
        m = stub(True)
        self.stop_and_release(m, chain_untangled=True)
        m.model.reset.assert_not_called()
        self.assertIsNone(m._release_stretch)   # slack: nothing to re-trigger on
        self.assertFalse(m._chain_untangled or m._hold_chain)

    def test_a_kept_chain_still_taut_at_the_release_stops_again_on_a_further_pull(self):
        m = stub(True)
        m._set_safety(STOP, "test")
        m._chain_untangled = True
        m._last_state = state(0.031)
        m._set_safety(NORMAL, "test")
        m.model.reset.assert_not_called()
        self.assertAlmostEqual(m._release_stretch, 0.031)

    def test_release_without_a_chain_untangle_resets_as_before(self):
        for flag in (True, False):
            m = stub(flag)
            self.stop_and_release(m, chain_untangled=False)
            m.model.reset.assert_called_once_with(Q)

    def test_a_new_stop_forgets_an_old_untangle(self):
        m = stub(True)
        m._chain_untangled = True
        m._set_safety(STOP, "test")
        self.assertFalse(m._chain_untangled)
        self.assertTrue(m._hold_chain)

    def test_the_real_robot_never_holds_the_chain(self):
        m = stub(True)
        m.mode = "real"
        m._set_safety(STOP, "test")
        self.assertFalse(m._hold_chain)


def params(name, default=None):
    return {"~trajectory_action_ns": "/mobipick"}.get(name, default)


class Untangle(unittest.TestCase):
    """_untangle itself, with the controller manager and the trajectory action stubbed"""
    def setUp(self):
        controllers = types.SimpleNamespace(controller=[types.SimpleNamespace(name="arm_velocity_controller",
                                                                              state="running")])
        proxies = {"list_controllers": mock.Mock(return_value=controllers),
                   "switch_controller": mock.Mock(return_value=types.SimpleNamespace(ok=True))}
        client = mock.Mock()
        client.wait_for_server.return_value = client.wait_for_result.return_value = True
        client.get_result.return_value = types.SimpleNamespace(error_code=0, error_string="")
        for patcher in (mock.patch.object(monitor.rospy, "get_param", params),
                        mock.patch.object(monitor.rospy, "get_time", lambda: 100.0),
                        mock.patch.object(monitor.rospy, "wait_for_service"),
                        mock.patch.object(monitor.rospy, "ServiceProxy",
                                          lambda name, _cls: proxies[name.rsplit("/", 1)[1]]),
                        mock.patch("actionlib.SimpleActionClient", return_value=client)):
            patcher.start()
            self.addCleanup(patcher.stop)
        self.client = client

    def run_untangle(self, flag):
        m = stub(flag)
        m._safety = STOP
        m._joint_names = {j: j for j in monitor.ARM_JOINTS}
        m._first_invalid = lambda _waypoints: None
        step = [("wrist_3", Q["wrist_3"] + math.radians(16.0))]
        m._planner.untangle_plan.return_value = (step, state(-0.063), True)
        m.model.untangle_plan.return_value = (step, state(-0.05), True)
        return m, m._untangle()

    def test_a_chain_untangle_marks_the_chain_and_keeps_it(self):
        m, res = self.run_untangle(True)
        self.assertTrue(res.success, res.message)
        self.client.send_goal.assert_called_once()
        self.assertTrue(m._chain_untangled)
        m.model.reset.assert_not_called()
        m._set_safety(NORMAL, "test")   # the release that follows
        m.model.reset.assert_not_called()

    def test_a_geometric_untangle_resets_the_model_as_before(self):
        m, res = self.run_untangle(False)
        self.assertTrue(res.success, res.message)
        self.assertFalse(m._chain_untangled)
        m.model.reset.assert_called_once_with(Q)

    def test_no_slack_moves_nothing(self):
        m = stub(True)
        m._safety = STOP
        m._joint_names = {j: j for j in monitor.ARM_JOINTS}
        m._planner.untangle_plan.return_value = ([], state(0.277), False)
        res = m._untangle()
        self.assertFalse(res.success)
        self.assertTrue(res.message.startswith("not moving: no slack within 90 deg"))
        self.client.send_goal.assert_not_called()
        self.assertFalse(m._chain_untangled)


class Tick(unittest.TestCase):
    """_tick's rest release (resets.step) waits while the stop holds the chain, and runs again after the release"""
    def setUp(self):
        for patcher in (mock.patch.object(monitor.rospy, "get_time", lambda: 100.0),
                        mock.patch.object(monitor.rospy, "get_param", lambda name, default=None: default)):
            patcher.start()
            self.addCleanup(patcher.stop)

    def ticking(self, flag):
        m = stub(flag)
        m._q_stamp, m._last_update, m._marker_t, m._armed, m._last_level = 100.0, None, 1e9, True, "ok"
        m.resets = mock.Mock(enabled=True)
        m.resets.step.return_value = None
        m._catch_up = lambda _stamp, dt: dt
        tick_state = types.SimpleNamespace(max_stretch=-0.06, level="ok", predicted_stop=False, to_dict=dict)
        m.model.update.return_value = tick_state
        m.pub_state, m.pub_stretch, m.pub_level, m._poll_safety = mock.Mock(), mock.Mock(), mock.Mock(), mock.Mock()
        return m

    def test_a_stop_holds_the_rest_release_until_it_is_released(self):
        m = self.ticking(True)
        m._set_safety(STOP, "test")
        m._tick(None)
        m.resets.step.assert_not_called()
        m.model.update.assert_called_once()   # the chain is still carried along
        m._set_safety(NORMAL, "test")
        m._tick(None)
        m.resets.step.assert_called_once()

    def test_without_the_flag_the_rest_release_runs_during_the_stop(self):
        m = self.ticking(False)
        m._set_safety(STOP, "test")
        m._tick(None)
        m.resets.step.assert_called_once()


if __name__ == "__main__":
    unittest.main()
