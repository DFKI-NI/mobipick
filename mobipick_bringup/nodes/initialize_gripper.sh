#!/bin/bash

# reset the gripper
rostopic pub --once /mobipick/gripper_hw/output robotiq_2f_gripper_control/Robotiq2FGripper_robot_output "{rACT: 0, rGTO: 0, rATR: 0, rPR: 0, rSP: 0, rFR: 0}"


# activate it, with max speed and force
rostopic pub --once /mobipick/gripper_hw/output robotiq_2f_gripper_control/Robotiq2FGripper_robot_output "{rACT: 1, rGTO: 0, rATR: 0, rPR: 0, rSP: 255, rFR: 255}"
