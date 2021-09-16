mobipick
========

[![pipeline status](https://git.ni.dfki.de/mobipick/mobipick/badges/noetic/pipeline.svg)](https://git.ni.dfki.de/mobipick/mobipick/commits/noetic)

This repo contains ROS configuration files (URDF description, Gazebo launch
files, MoveIt config, bringup launch files) for the Mobipick
robot (MiR 100 base, UR5 arm, Robotiq 2 Finger Gripper + Force-Torque Sensor,
Orbbec Astra Mini S 3D camera).

![](doc/img/mobipick_v0.jpg)


Package overview
----------------

* `mobipick_bringup`: Launch and configuration files for the real Mobipick robot
* `mobipick_description`: URDF description of the robot
* `mobipick_gazebo`: Simulation specific launch and configuration files
* `mobipick_moveit_config`: MoveIt! launch and configuration files
* `mobipick_pick_n_place`: Some demo nodes that show off the MoveIt capabilities of the robot


Installation
------------

First, [install ROS](http://wiki.ros.org/ROS/Installation). Then:

```bash
# create a catkin workspace and clone all required ROS packages
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
git clone -b noetic git@git.ni.dfki.de:mobipick/mobipick.git

mobipick/install-deps.sh
mobipick/build.sh
```

If you have a physical pico flexx camera attached to this PC, also follow the
installation instructions of the
[pico_flexx_driver](https://github.com/code-iai/pico_flexx_driver). These
cannot be integrated into the instructions above because they require a manual
download step.

You can also optionally install the dependencies in
dependencies-optional.rosinstall; simply uncomment the relevant line in
`install-deps.sh` before running it.

You should add the following line to the end of your `~/.bashrc`, and then
close and reopen all terminals:

```bash
source ~/catkin_ws/devel/setup.bash
```

**Note:** Since we are using the `devel` space, some nodes (for example in
Universal_Robots_ROS_Driver) still have the `#!/usr/bin/env python` shebang
line. If this leads to an error, simply install the package
`python-is-python3`.


Quick start
-----------

The following examples describe how to use the robot in simulation. For more
information on how to use the real mobipick robot have a look at the
[README.md in `mobipick_bringup`](mobipick_bringup/README.md).

### Pick + Place with move_base demo (Berghoffstraße)

```bash
roslaunch mobipick_gazebo mobipick_moelk.launch
rosservice call /gazebo/unpause_physics
roslaunch mir_gazebo fake_localization.launch __ns:="mobipick" odom_frame_id:="mobipick/odom" base_frame_id:="mobipick/base_footprint"
roslaunch mir_navigation start_planner.launch map_file:=$(rospack find pbr_maps)/maps/moelk/pbr_robot_lab.yaml prefix:="mobipick/"
roslaunch mobipick_moveit_config moveit_planning_execution.launch use_pointcloud:=true simulation:=true
rviz -d $(rospack find mir_navigation)/rviz/navigation.rviz __ns:="mobipick"
roslaunch mobipick_pick_n_place mobipick_pick_n_place.launch simulation:=true
rosservice call /mobipick/continue_statemachine
```

There exists also an analogous demo using [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)
instead of a state machine. The implementation doesn't support pausing as easily, however,
so this functionality is removed.
The graphical editor [Groot](https://github.com/BehaviorTree/Groot) is directly supported in this demo.
To see its visualization, run `rosrun groot Groot` and connect the monitor with default settings
after the behavior tree started.

```bash
roslaunch mobipick_gazebo mobipick_moelk.launch
rosservice call /gazebo/unpause_physics
roslaunch mir_gazebo fake_localization.launch __ns:="mobipick" odom_frame_id:="mobipick/odom" base_frame_id:="mobipick/base_footprint"
roslaunch mir_navigation start_planner.launch map_file:=$(rospack find pbr_maps)/maps/moelk/pbr_robot_lab.yaml prefix:="mobipick/"
roslaunch mobipick_moveit_config moveit_planning_execution.launch use_pointcloud:=true simulation:=true
rviz -d $(rospack find mir_navigation)/rviz/navigation.rviz __ns:="mobipick"
roslaunch mobipick_pick_n_place moveit_macros.launch
roslaunch mobipick_pick_n_place mobipick_pick_n_place_bt.launch simulation:=true
rosrun groot Groot
```

Optionally execute `rosservice call /mobipick/simulate_user_interaction` during
the user interaction step to fake taking the power drill.

### Pick + Place demo (Gazebo)

```bash
roslaunch mobipick_gazebo mobipick_mrk_lab_world.launch
rosservice call /gazebo/unpause_physics   # or click the "start" button in the Gazebo GUI
roslaunch mir_gazebo fake_localization.launch __ns:="mobipick"
roslaunch mobipick_moveit_config moveit_planning_execution.launch use_pointcloud:=true simulation:=true
roslaunch mobipick_moveit_config moveit_rviz.launch    # not required, just for visualization
roslaunch mobipick_pick_n_place mobipick_pick_n_place.launch
```

[![Mobipick pick + place demo (Gazebo)](https://i.vimeocdn.com/video/683635424.jpg?mw=640)](https://vimeo.com/256064111)

(Click image to see video)


### Pick + Place demo (MoveIt! demo mode)

```bash
roslaunch mobipick_moveit_config demo.launch
roslaunch mobipick_pick_n_place mobipick_pick_n_place.launch simulation:=true object_source:=static
rosservice call /mobipick/continue_statemachine
```

* Make sure that the parameter `use_sim_time` is set to `false`.
* You can play around with different planning pipelines, which you can select
  in the the "MotionPlanning" pane in RViz under "Context".
  - `chomp` (with our current configuration) is very slow (takes 50 s on my
    machine). `chomp` is the default *in RViz* (because RViz uses alphabetical
    sorting).
  - `ompl` is much faster and is used in the `pick_n_place` demos. `ompl` is
    the default *in MoveIt*.
  - `pilz_industrial_motion_planner` is a simplistic motion planning pipeline
    that works for easy motions. It is somewhat hard to control from RViz, but
    the `PTP` planner works.

[![Mobipick pick + place demo (MoveIt! demo mode)](https://i.vimeocdn.com/video/683635444.jpg?mw=640)](https://vimeo.com/256064108)

(Click image to see video)


### move_base demo (maze world)

```bash
roslaunch mobipick_gazebo mobipick_maze_world.launch
rosservice call /gazebo/unpause_physics   # or click the "start" button in the Gazebo GUI
roslaunch mir_gazebo fake_localization.launch delta_x:=-10.0 delta_y:=-10.0
# or alternatively: roslaunch mir_gazebo fake_localization.launch delta_x:=-10.0 delta_y:=-10.0
rosrun mobipick_gazebo move_arm_to_home
roslaunch mir_navigation start_planner.launch \
    map_file:=$(rospack find mir_gazebo)/maps/maze.yaml \
    virtual_walls_map_file:=$(rospack find mir_gazebo)/maps/maze_virtual_walls.yaml
rviz -d $(rospack find mir_navigation)/rviz/navigation.rviz
```

Now, you can use the "2D Nav Goal" tool in RViz to set a navigation goal for move_base.


### move_base demo (MRK lab world)

```bash
roslaunch mobipick_gazebo mobipick_mrk_lab_world.launch
rosservice call /gazebo/unpause_physics   # or click the "start" button in the Gazebo GUI
roslaunch mir_gazebo fake_localization.launch __ns:="mobipick" odom_frame_id:="mobipick/odom" base_frame_id:="mobipick/base_footprint"
ROS_NAMESPACE=mobipick rosrun mobipick_gazebo move_arm_to_home
roslaunch mir_navigation start_planner.launch \
    map_file:=$(rospack find mobipick_gazebo)/maps/rh5_mrk_lab.yaml prefix:="mobipick/"
rviz -d $(rospack find mir_navigation)/rviz/navigation.rviz __ns:="mobipick"
```

Now, you can use the "2D Nav Goal" tool in RViz to set a navigation goal for move_base.


### Pick + Place with demo (Smart Factory)

```bash
roslaunch mobipick_gazebo mobipick_smart_factory.launch
rosservice call /gazebo/unpause_physics
roslaunch mir_gazebo fake_localization.launch __ns:="mobipick" odom_frame_id:="mobipick/odom" base_frame_id:="mobipick/base_footprint" delta_yaw:=1.57 delta_x:=0.2
roslaunch mir_navigation start_planner.launch map_file:=$(rospack find mobipick_gazebo)/maps/smart_factory_merged.yaml prefix:="mobipick/"
roslaunch mobipick_moveit_config moveit_planning_execution.launch use_pointcloud:=true simulation:=true
rviz -d $(rospack find mir_navigation)/rviz/navigation.rviz __ns:="mobipick" world:="smart_factory"
roslaunch mobipick_pick_n_place mobipick_pick_n_place.launch simulation:=true world:="smart_factory"
rosservice call /mobipick/continue_statemachine
```

### Rosbridge with websockets

```bash
roslaunch rosbridge_server rosbridge_websocket.launch port:=9090
```

### Using the arm velocity controller

If you want to use the velocity controller for the arm, add
`arm_velocity_controller:=true` to any of the launch files, like so:

```bash
roslaunch mobipick_gazebo mobipick_table_world.launch arm_velocity_controller:=true
rosservice call /gazebo/unpause_physics   # or click the "start" button in the Gazebo GUI
```

Now you can send velocity commands to the arm directly, like this:

```bash
rostopic pub /arm_velocity_controller/command std_msgs/Float64MultiArray "data: [0.4, 0.1, -0.1, 0, 0, 0]"
```

In this configuration, MoveIt won't work however, since MoveIt requires the
joint trajectory controller instead of the velocity controller (because MoveIt
is a joint trajectory client).

You can switch to the arm trajectory controller like this:

```bash
rosrun controller_manager controller_manager stop arm_velocity_controller
rosrun controller_manager controller_manager start arm_controller
```

This also works the other way around, e.g. if you didn't add the
`arm_velocity_controller:=true` argument to the launch file:

```bash
rosrun controller_manager controller_manager stop arm_controller
rosrun controller_manager controller_manager start arm_velocity_controller
```

If you prefer a graphical interface, try this:

```bash
rosrun rqt_controller_manager rqt_controller_manager
```

Updating the SRDF
-----------------

Whenever the URDF of the robot changes, the SRDF has to be updated as well.
Since we changed the URDF to xacro, the following process is necessary:

```bash
roscd mobipick_moveit_config/config/
xacro mobipick.srdf.xacro tf_prefix:=mobipick > mobipick.srdf
roslaunch mobipick_moveit_config setup_assistant.launch

sed -i 's/mobipick\//${tf_prefix}/g' mobipick.srdf
meld mobipick.srdf mobipick.srdf.xacro   # compare and apply all relevant changes
rm mobipick.srdf
```


pre-commit Formatting Checks
----------------------------

This repo has a [pre-commit](https://pre-commit.com/) check that runs in CI.
You can use this locally and set it up to run automatically before you commit
something. To install, use pip:

```bash
pip3 install --user pre-commit
```

To run over all the files in the repo manually:

```bash
pre-commit run -a
```

To run pre-commit automatically before committing in the local repo, install the git hooks:

```bash
pre-commit install
```


Troubleshooting
---------------

### Problem: Arm doesn't move

Sometimes the arm doesn't move, and the following errors appear in the log:

```text
[ WARN] [1625228178.110222489]: Controller scaled_pos_traj_controller failed with error INVALID_GOAL:  (/mobipick/move_group)
[ WARN] [1625228178.110321441]: Controller handle scaled_pos_traj_controller reports status FAILED (/mobipick/move_group)
[ INFO] [1625228178.110378244]: Completed trajectory execution with status FAILED ... (/mobipick/move_group)
```

This happens every time that `roslaunch mobipick_bringup
mobipick_bringup_control.launch` is restarted, because the program on the UR5
robot must be started **after** that launch file is run.

**Solution:** First make sure that `mobipick_bringup_control.launch` is
running, then stop and restart the UR program on the robot. You can do this by
pressing "stop" and then "start on the teaching pendant. Alternatively, call
the following services:

```bash
rosservice call /mobipick/ur_hardware_interface/dashboard/stop
rosservice call /mobipick/ur_hardware_interface/dashboard/play
```
