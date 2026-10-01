# mobipick_gazebo_hw

Simulation-only `gazebo_ros_control` hardware interface for the Mobipick (#250).

`gazebo_ros_control`'s `DefaultRobotHWSim::readSim` keeps `joint_position += shortest_angular_distance(joint_position,
gazebo_angle)` for every joint that is not prismatic. If the arm is flung by more than pi between two controller updates
(for example by a Gazebo `set_model_state` teleport of the whole robot), the shortest step is the wrong way round and
`/joint_states` gains or loses a whole turn for good (wrist_3 179.9 -> 539.9 -> 15299.9 deg) although Gazebo's own angle is
right. MoveIt and the cable model then see a wound arm. A revolute joint with position limits (every UR arm joint)
can never legitimately leave its limit range, so `MobipickRobotHWSim` reports Gazebo's absolute angle for it; continuous
joints keep the accumulation. Everything else is `DefaultRobotHWSim`.

Selected by `robot_hw_sim` of the `controller_plugin_gazebo` macro in `mobipick_description/urdf/common.gazebo.xacro`
(default `mobipick_gazebo_hw/MobipickRobotHWSim`; `gazebo_ros_control/DefaultRobotHWSim` restores the stock behaviour). The
real robot does not use it. Build it with the workspace (`catkin build mobipick_gazebo_hw`) before the next sim start.

Test (one-joint Gazebo, no robot stack): `test/hw_sim_test.py` with `test/hw_sim_test.urdf`: normal motion is identical, a jump of
more than 180 degrees winds the stock interface (-360 deg) and not this one.
