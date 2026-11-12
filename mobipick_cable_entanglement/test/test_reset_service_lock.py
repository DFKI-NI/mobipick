"""The monitor's ~reset service rebuilds the live chain under the model lock, like the tick's update and the release
after a stop: the service runs in its own thread, and an update during the rebuild read a half-built chain (a failed
chain, a jump to +75 % stretch and a sim protective stop with the arm standing still). Stubs instead of a ROS master
(needs rospy: the Mobipick image, python3 -m unittest)."""
import importlib.util
import os
import sys
import threading
import unittest
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
spec = importlib.util.spec_from_file_location(
    "cable_entanglement_monitor", os.path.join(HERE, "..", "nodes", "cable_entanglement_monitor.py"))
monitor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor)

Q = {j: 0.1 * i for i, j in enumerate(monitor.ARM_JOINTS)}


def stub():
    m = monitor.CableEntanglementMonitor.__new__(monitor.CableEntanglementMonitor)
    m._lock = threading.Lock()
    m._model_lock = threading.RLock()
    m._q = dict(Q)
    m.model = mock.Mock()
    m.resets = mock.Mock()
    m._armed = False
    return m


class ResetService(unittest.TestCase):
    def setUp(self):
        patcher = mock.patch.object(monitor.rospy, "get_time", lambda: 12.5)
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_reset_rebuilds_the_chain_under_the_model_lock(self):
        m = stub()
        held = []
        m.model.reset.side_effect = lambda q: held.append(m._model_lock._is_owned())
        m.resets.forget_rest.side_effect = lambda *a: held.append(m._model_lock._is_owned())
        r = m._srv_reset(None)
        self.assertTrue(r.success)
        self.assertEqual(held, [True, True])
        m.model.reset.assert_called_once_with(Q)
        self.assertTrue(m._armed)

    def test_a_tick_holding_the_lock_delays_the_reset(self):
        m = stub()
        m._model_lock.acquire()   # a tick in the middle of its update
        done = threading.Event()
        t = threading.Thread(target=lambda: (m._srv_reset(None), done.set()), daemon=True)
        t.start()
        self.assertFalse(done.wait(0.3))
        m.model.reset.assert_not_called()
        m._model_lock.release()
        self.assertTrue(done.wait(2.0))
        m.model.reset.assert_called_once_with(Q)

    def test_no_joint_states_no_reset(self):
        m = stub()
        m._q = None
        r = m._srv_reset(None)
        self.assertFalse(r.success)
        m.model.reset.assert_not_called()


if __name__ == "__main__":
    unittest.main()
