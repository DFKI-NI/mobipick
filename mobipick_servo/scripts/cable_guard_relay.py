#!/usr/bin/env python3
"""Cable guard relay between MoveIt Servo and the arm controller (#200).

    roslaunch mobipick_servo servo.launch   (starts it next to servo_server)

Servo publishes its joint targets (trajectory_msgs/JointTrajectory) on ~command_in (servo_server/command_unguarded);
each one goes on to ~command_out (arm_controller/command) only when mobipick_servo.guard allows it with the cable model
of mobipick_cable_entanglement (geometry only; ~max_stretch -0.115 like grasplan's MTC guard; a wrist_3 window of
~wrist_3_half_window_deg 170 around the cable carrier's neutral angle). A refused target is replaced by one hold at the
current
joint state and reported on ~blocked (std_msgs/String). Without the cable model (mobipick_cable_entanglement not built) nothing is
forwarded unless ~cable_guard is false.

Servo output is only forwarded while commands arrive (within ~input_window_s, 0.3 s, of the last twist or joint jog on
~twist_in / ~joint_in): moveit_servo 1.1.16 keeps publishing holds at the servo rate after a client stops without a
final zero command (it remembers the last command as non-zero), and on the shared arm_controller those holds replace
every MoveIt trajectory (sim 2026-09-29).
"""
import math
import os
import threading
import time

import rospkg
import rospy
import yaml
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from mobipick_servo import guard


class Relay:
    def __init__(self):
        self.enabled = bool(rospy.get_param('~cable_guard', True))
        self.margin = float(rospy.get_param('~max_stretch', -0.115))
        self.half_window = math.radians(float(rospy.get_param('~wrist_3_half_window_deg', 170.0)))
        self.pub = rospy.Publisher(rospy.get_param('~command_out', 'arm_controller/command'), JointTrajectory, queue_size=1)
        self.blocked_pub = rospy.Publisher('~blocked', String, queue_size=1)
        self.lock = threading.Lock()
        self.joints = {}
        self.model = None
        self.window = None
        self.blocked = False
        self.input_window = float(rospy.get_param('~input_window_s', 0.3))
        self.last_input = None  # time.monotonic() of the last twist or joint jog to servo
        self.checks = 0
        self.check_s = 0.0
        if self.enabled:
            self._load_model()
        rospy.Subscriber(rospy.get_param('~joint_states', 'joint_states'), JointState, self._on_joints, queue_size=1)
        rospy.Subscriber(rospy.get_param('~twist_in', 'servo_server/delta_twist_cmds'), TwistStamped, self._on_input)
        rospy.Subscriber(rospy.get_param('~joint_in', 'servo_server/delta_joint_cmds'), JointJog, self._on_input)
        rospy.Subscriber(
            rospy.get_param('~command_in', 'servo_server/command_unguarded'), JointTrajectory, self._on_command, queue_size=1
        )

    def _load_model(self):
        try:
            from mobipick_cable_entanglement.cable_model import CableModel, config_from_dict  # separate package

            path = rospy.get_param('~cable_model_config', '') or os.path.join(
                rospkg.RosPack().get_path('mobipick_cable_entanglement'), 'config', 'cable_model.yaml'
            )
            with open(path) as f:
                cfg = config_from_dict(yaml.safe_load(f))
            cfg.cable_carrier.enabled = False
            self.model = CableModel(rospy.get_param('robot_description'), cfg)
            wrist_3 = [wj for span in cfg.spans for wj in span.joints if wj.joint.endswith('ur5_wrist_3_joint')]
            neutral = wrist_3[0].neutral if wrist_3 else math.pi - 0.25
            self.window = (neutral - self.half_window, neutral + self.half_window)
            rospy.loginfo(
                'cable guard: model %s, max stretch %+.3f, wrist_3 %.0f +- %.0f deg',
                path,
                self.margin,
                math.degrees(neutral),
                math.degrees(self.half_window),
            )
        except Exception as exc:  # noqa: BLE001 - fail safe: nothing is forwarded
            rospy.logerr(
                'cable guard: no cable model (%s): servo commands are NOT forwarded (~cable_guard false relays them '
                'unchecked)',
                exc,
            )

    def _on_input(self, _msg):
        self.last_input = time.monotonic()

    def _on_joints(self, msg):
        with self.lock:
            self.joints.update(zip(msg.name, msg.position))

    def _hold(self, names, now):
        hold = JointTrajectory(joint_names=list(names))
        hold.points = [JointTrajectoryPoint(positions=[now[n] for n in names], time_from_start=rospy.Duration(0.1))]
        self.pub.publish(hold)

    def _on_command(self, msg):
        if not msg.points:
            return
        if self.last_input is None or time.monotonic() - self.last_input > self.input_window:
            return  # servo's hold stream after the operator stopped: leave the controller to MoveIt
        if not self.enabled:
            self.pub.publish(msg)
            return
        with self.lock:
            now = {n: self.joints[n] for n in msg.joint_names if n in self.joints}
        if self.model is None or len(now) != len(msg.joint_names):
            rospy.logwarn_throttle(5.0, 'cable guard: no cable model or no joint state yet: servo command dropped')
            return
        target = max(msg.points, key=lambda p: p.time_from_start.to_sec())
        started = time.perf_counter()
        verdict = guard.check(
            now,
            dict(zip(msg.joint_names, target.positions)),
            lambda q: self.model.evaluate(q).max_stretch,
            self.margin,
            self.window,
        )
        self.checks += 1
        self.check_s += time.perf_counter() - started
        if verdict.forward:
            if self.blocked:
                rospy.loginfo('cable guard: released (%s)', verdict.reason)
                self.blocked = False
            self.pub.publish(msg)
            return
        if not self.blocked:
            rospy.logwarn('cable guard: servo command refused: %s', verdict.reason)
            self._hold(msg.joint_names, now)
            self.blocked = True
        self.blocked_pub.publish(String(data=verdict.reason))

    def report(self, _event):
        if self.checks:
            rospy.loginfo('cable guard: %d checks, %.1f ms each', self.checks, 1000.0 * self.check_s / self.checks)
            self.checks, self.check_s = 0, 0.0


if __name__ == '__main__':
    rospy.init_node('cable_guard_relay')
    relay = Relay()
    rospy.Timer(rospy.Duration(30.0), relay.report)
    rospy.spin()
