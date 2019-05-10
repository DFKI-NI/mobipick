#!/bin/bash

# wait for the gripper driver to come up, by waiting for a gripper status message
# ("gripper_hw/input" is input the node gets from the gripper, "output" are commands send to the gripper)
rostopic echo -n 1 /mobipick/gripper_hw/input

# reset the gripper
rostopic pub --once /mobipick/gripper_hw/output robotiq_2f_gripper_control/Robotiq2FGripper_robot_output "{rACT: 0, rGTO: 0, rATR: 0, rPR: 0, rSP: 0, rFR: 0}"


# activate it, with max speed and force
rostopic pub --once /mobipick/gripper_hw/output robotiq_2f_gripper_control/Robotiq2FGripper_robot_output "{rACT: 1, rGTO: 0, rATR: 0, rPR: 0, rSP: 255, rFR: 255}"
