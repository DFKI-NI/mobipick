#!/usr/bin/env python3

import sys

import rospy
from std_srvs.srv import Trigger
from ur_dashboard_msgs.msg import SafetyMode, RobotMode, ProgramState
from ur_dashboard_msgs.srv import GetSafetyMode, GetRobotMode, GetProgramState

rospy.init_node('reset_arm')


def call_service(name, service_class, timeout=5.0):
    service_name = f'ur_hardware_interface/dashboard/{name}'
    rospy.wait_for_service(service_name, timeout)
    rospy.logdebug(f'calling {service_name}')
    return rospy.ServiceProxy(service_name, service_class)()


# if safety mode not NORMAL, unlock_protective_stop
resp = call_service('get_safety_mode', GetSafetyMode, timeout=30.0)
rospy.loginfo(resp.answer)

while resp.safety_mode.mode == SafetyMode.ROBOT_EMERGENCY_STOP:
    rospy.logwarn('Emergency stop activated! Release the emergency stop.')
    rospy.sleep(1.0)
    rospy.loginfo('trying again')
    resp = call_service('get_safety_mode', GetSafetyMode)
    rospy.loginfo(resp.answer)

while resp.safety_mode.mode == SafetyMode.SAFEGUARD_STOP:
    rospy.loginfo(
        'Robot is in SAFEGUARD_STOP (happens right after ROBOT_EMERGENCY_STOP). '
        'Waiting for transition to PROTECTIVE_STOP/NORMAL.'
    )
    resp = call_service('get_safety_mode', GetSafetyMode)
    rospy.logdebug(resp.answer)

if resp.safety_mode.mode == SafetyMode.PROTECTIVE_STOP:
    while True:
        rospy.logwarn("WARNING! Unlocking protective stop!")
        resp = call_service('unlock_protective_stop', Trigger)
        # This can fail
        rospy.loginfo(resp.message)
        if resp.success:
            resp = call_service('get_safety_mode', GetSafetyMode, timeout=30.0)
            rospy.loginfo(resp.answer)
            break
        rospy.sleep(1.0)
        rospy.loginfo('trying again')

# wait until safety mode NORMAL
while resp.safety_mode.mode != SafetyMode.NORMAL:
    rospy.loginfo('still waiting for SafetyMode NORMAL...')
    rospy.sleep(1.0)
    resp = call_service('get_safety_mode', GetSafetyMode, timeout=30.0)
    rospy.loginfo(resp.answer)

################################################################################

# if robot not RUNNING, power on + brake release
resp = call_service('get_robot_mode', GetRobotMode)
rospy.loginfo(resp.answer)

if resp.robot_mode.mode == RobotMode.POWER_OFF:
    resp = call_service('power_on', Trigger)
    rospy.loginfo(resp.message)

    resp = call_service('get_robot_mode', GetRobotMode)
    rospy.loginfo(resp.answer)

# wait until IDLE
while resp.robot_mode.mode in [RobotMode.POWER_OFF, RobotMode.POWER_ON]:
    resp = call_service('get_robot_mode', GetRobotMode)
    rospy.logdebug(resp.answer)

if resp.robot_mode.mode == RobotMode.IDLE:
    resp = call_service('brake_release', Trigger)
    rospy.loginfo(resp.message)
    resp = call_service('get_robot_mode', GetRobotMode)
    rospy.loginfo(resp.answer)

# wait until RUNNING
while resp.robot_mode.mode == RobotMode.IDLE:
    resp = call_service('get_robot_mode', GetRobotMode)
    rospy.logdebug(resp.answer)

if resp.robot_mode.mode != RobotMode.RUNNING:
    rospy.logfatal(f'Unhandled robot mode: {resp.robot_mode.mode}!')
    sys.exit(-1)

################################################################################

# if program state not STOPPED: stop, then always play

resp = call_service('program_state', GetProgramState)
rospy.loginfo(resp.answer)

if resp.state.state != ProgramState.STOPPED:
    resp = call_service('stop', Trigger)
    rospy.loginfo(resp.message)
    resp = call_service('program_state', GetProgramState)
    rospy.loginfo(resp.answer)

# wait until STOPPED
while resp.state.state != ProgramState.STOPPED:
    resp = call_service('program_state', GetProgramState)
    rospy.loginfo(resp.answer)
    rospy.sleep(1.0)
    rospy.loginfo('still waiting for the program to stop')

# wait a little bit longer just to make sure we don't "press play twice" and crash the dashboard
rospy.sleep(2.0)

resp = call_service('play', Trigger)
rospy.loginfo(resp.message)
