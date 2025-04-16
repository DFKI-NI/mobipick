#!/usr/bin/env python3

import sys

import rospy
import moveit_commander

rospy.init_node('reset_moveit')

moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
gripper = getattr(robot, 'gripper')
scene = moveit_commander.PlanningSceneInterface()

objects_of_interest = rospy.get_param('/pick_pose_selector_node/objects_of_interest', [])

# detach objects from gripper
for attached_object in scene.get_attached_objects().keys():
    gripper.detach_object(name=attached_object)

# iterate over all object in the scene
for item in scene.get_known_object_names():
    # only delete objects of interest from the scene if there are any
    if len(objects_of_interest) > 0:
        if item[:-2] in objects_of_interest:
            scene.remove_world_object(item)
    else:
        scene.remove_world_object(item)
