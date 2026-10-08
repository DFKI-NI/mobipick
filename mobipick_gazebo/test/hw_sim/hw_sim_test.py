#!/usr/bin/env python3
"""DefaultRobotHWSim vs MobipickRobotHWSim in a one-joint Gazebo model (limits 30..275 deg like wrist_3).
(a) normal motion: the joint is moved in 2 deg steps (set_model_configuration, 20 ms apart) from 40 to 270 and back: both
    interfaces must report the same angle as Gazebo (max difference printed).
(b) big jumps: 40 -> 270 -> 40 -> ... in one step (more than 180 deg between two controller updates): the default interface
    reports -90 for 270 (a -360 error), the Mobipick one must report 270.
Prints one line per check and PASS/FAIL for the interface under test (argument: label)."""
import math, sys, time
import rospy
from gazebo_msgs.srv import GetJointProperties, SetModelConfiguration
from sensor_msgs.msg import JointState

label = sys.argv[1]
expect_fixed = sys.argv[2] == 'fixed'
rospy.init_node('hw_sim_test')
rospy.wait_for_service('/gazebo/set_model_configuration', 60)
setc = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
get = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
state = {}
rospy.Subscriber('/joint_states', JointState, lambda m: state.update(dict(zip(m.name, m.position))))

def to(deg):
    setc('tj', 'robot_description', ['wrist'], [math.radians(deg)])

def js():
    time.sleep(0.25)
    return math.degrees(state.get('wrist', float('nan')))

def gz():
    return math.degrees(get('wrist').position[0])

while 'wrist' not in state:
    time.sleep(0.1)
to(40.0); time.sleep(1.0)
worst = 0.0
for a in list(range(40, 271, 2)) + list(range(270, 39, -2)):
    to(float(a)); time.sleep(0.02)
    time.sleep(0.03)
    worst = max(worst, abs(math.degrees(state['wrist']) - gz()))
print(f'{label}: (a) normal motion 40->270->40 in 2 deg steps: max |joint_states - gazebo| = {worst:.2f} deg')
bad = 0
for target in (270.0, 40.0, 270.0, 40.0, 270.0):
    to(target)
    r = js()
    ok = abs(r - target) < 1.0
    bad += not ok
    print(f'{label}: (b) jump to {target:5.0f}: joint_states {r:8.1f}  gazebo {gz():8.1f}  {"ok" if ok else "WOUND (off by " + format(r - target, ".0f") + ")"}')
passed = (worst < 1.0) and ((bad == 0) if expect_fixed else (bad > 0))
print(f'{label}: {"PASS" if passed else "FAIL"} ({"must not wind" if expect_fixed else "stock interface is expected to wind"})')
