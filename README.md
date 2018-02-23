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
* `mobipick_gazebo`: Smulation specific launch and configuration files
* `mobipick_moveit_config`: MoveIt! launch and configuration files
* `mobipick_navigation`: move_base launch and configuration files
* `mobipick_pick_n_place`: Some demo nodes that show off the MoveIt capabilities of the robot


Installation
------------

See the build step in [`.gitlab-ci.yml`](.gitlab-ci.yml).


Quick start
-----------

### Pick + Place demo (Gazebo)

```bash
roslaunch mobipick_gazebo mobipick_table_world.launch
rosservice call /gazebo/unpause_physics   # or click the "start" button in the Gazebo GUI
roslaunch mobipick_gazebo fake_localization.launch
roslaunch mobipick_moveit_config moveit_planning_execution.launch use_pointcloud:=true
roslaunch mobipick_moveit_config moveit_rviz.launch config:=true   # not required, just for visualization
roslaunch mobipick_pick_n_place mobipick_pick_n_place.launch
```

[![Mobipick pick + place demo (Gazebo)](https://i.vimeocdn.com/video/683635424.jpg?mw=640)](https://vimeo.com/256064111)

(Click image to see video)


### Pick + Place demo (MoveIt! demo mode)

```bash
roslaunch mobipick_moveit_config demo.launch
roslaunch mobipick_pick_n_place mobipick_pick_n_place.launch object_source:=static
```

[![Mobipick pick + place demo (MoveIt! demo mode)](https://i.vimeocdn.com/video/683635444.jpg?mw=640)](https://vimeo.com/256064108) 

(Click image to see video)


### move_base demo (maze world)

```bash
roslaunch mobipick_gazebo mobipick_maze_world.launch
rosservice call /gazebo/unpause_physics   # or click the "start" button in the Gazebo GUI
roslaunch mobipick_gazebo fake_localization.launch delta_x:=-10.0 delta_y:=-10.0
roslaunch mobipick_navigation start_planner.launch \
    map_file:=$(rospack find mobipick_gazebo)/maps/maze.yaml \
    virtual_walls_map_file:=$(rospack find mobipick_gazebo)/maps/maze_virtual_walls.yaml \
    local_planner:=dwa
rviz -d $(rospack find mobipick_navigation)/rviz/navigation.rviz
```

Now, you can use the "2D Nav Goal" tool in RViz to set a navigation goal for move_base. 


### move_base demo (AVZ world)

```bash
roslaunch mobipick_gazebo mobipick_avz_world.launch
rosservice call /gazebo/unpause_physics   # or click the "start" button in the Gazebo GUI
roslaunch mobipick_gazebo fake_localization.launch delta_x:=-14.70 delta_y:=-15.0 delta_yaw:=-1.57
roslaunch mobipick_navigation start_planner.launch map_file:=$(rospack find uos_maps)/maps/avz5floor_gazebo.yaml local_planner:=dwa
rviz -d $(rospack find mobipick_navigation)/rviz/navigation.rviz
```

Now, you can use the "2D Nav Goal" tool in RViz to set a navigation goal for move_base. 
