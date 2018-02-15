mobipick
========

[![pipeline status](https://git.hb.dfki.de/mobipick/mobipick/badges/kinetic/pipeline.svg)](https://git.hb.dfki.de/mobipick/mobipick/commits/kinetic)

This repo contains ROS configuration files (URDF description, Gazebo launch
files, MoveIt config, move_base config, bringup launch files) for the Mobipick
robot (MiR 100 base, UR5 arm, Robotiq 2 Finger Gripper + Force-Torque Sensor,
Orbbec Astra Mini S 3D camera).

![](doc/img/mobipick_v0.jpg)


Package overview
----------------

* `mobipick_description`: URDF description of the robot
* `mobipick_gazebo`: simulation specific configuration + launch files
* `mobipick_moveit_config`: MoveIt! configuration files
* `mobipick_pick_n_place`: Some demo nodes that show off the MoveIt capabilities of the robot


Installation
------------

See the build step in [`.gitlab-ci.yml`](.gitlab-ci.yml).


Quick start
-----------

### Pick + Place demo (Gazebo)

```bash
roslaunch mobipick_gazebo mobipick.gazebo.launch
rosservice call /gazebo/unpause_physics   # or click the "start" button in the Gazebo GUI
roslaunch mobipick_gazebo fake_localization.launch
roslaunch mobipick_moveit_config moveit_planning_execution.launch use_pointcloud:=true
roslaunch mobipick_moveit_config moveit_rviz.launch config:=true   # not required, just for visualization
roslaunch mobipick_pick_n_place mobipick_pick_n_place.launch
```

### Pick + Place demo (MoveIt! demo mode)

```bash
roslaunch mobipick_moveit_config demo.launch
roslaunch mobipick_pick_n_place mobipick_pick_n_place.launch object_source:=static
```
