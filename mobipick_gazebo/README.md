# mobipick_gazebo

Simulation specific launch and configuration files for the Mobipick robot: Gazebo worlds, maps, the simulated objects
and tables, the controller configuration, the simulated UR dashboard (rqt plugin and node) and the simulation-only
hardware interface below. See the repository README for how to launch the simulation.

## Simulated hardware interface (`mobipick_gazebo/MobipickRobotHWSim`)

Simulation-only `gazebo_ros_control` hardware interface for the Mobipick (`src/hw/mobipick_robot_hw_sim.cpp`, plugin
description `robot_hw_sim_plugins.xml`, library `libmobipick_robot_hw_sim`).

`gazebo_ros_control`'s `DefaultRobotHWSim::readSim` keeps `joint_position += shortest_angular_distance(joint_position,
gazebo_angle)` for every joint that is not prismatic. If the arm is flung by more than pi between two controller updates
(for example by a Gazebo `set_model_state` teleport of the whole robot), the shortest step is the wrong way round and
`/joint_states` gains or loses a whole turn for good (wrist_3 179.9 -> 539.9 -> 15299.9 deg) although Gazebo's own angle is
right. MoveIt and the cable model then see a wound arm. A revolute joint with position limits (every UR arm joint)
can never legitimately leave its limit range, so `MobipickRobotHWSim` reports Gazebo's absolute angle for it; continuous
joints keep the accumulation. Everything else is `DefaultRobotHWSim`.

Selected by `robot_hw_sim` of the `controller_plugin_gazebo` macro in `mobipick_description/urdf/common.gazebo.xacro`
(default `mobipick_gazebo/MobipickRobotHWSim`; `gazebo_ros_control/DefaultRobotHWSim` restores the stock behaviour). The
real robot does not use it. Build it with the workspace (`catkin build mobipick_gazebo`) before the next sim start.

Test (one-joint Gazebo, no robot stack): `test/hw_sim/hw_sim_test.py` with `test/hw_sim/hw_sim_test.urdf` (replace
`ROBOT_HW_SIM` in the urdf by the interface under test): normal motion is identical, a jump of more than 180 degrees
winds the stock interface (-360 deg) and not this one.

## Robot colours in a multi-robot sim (`robot_colours`)

With several Mobipicks in one Gazebo (`moelk_tables_x2_spawn_sim_objects.launch`, `..._x3_...`) every robot is told
apart by a colour that follows its order: robot 1 blue, robot 2 orange, robot 3 green, robot 4 purple. The table lives
in one file, `mobipick_description/config/robot_colours.yaml`, read by the robot description (xacro) and by the Python
module `mobipick_gazebo.robot_colours` (display names such as "orange Mobipick", GUI dots,
`python3 -m mobipick_gazebo.robot_colours <namespace|index> [name|hex|rgba|gazebo_material|display_name|title]`).

The world launches `moelk_tables_spawn_sim_objects.launch`, `cic_tables_spawn_sim_objects.launch`,
`moelk_tables_x2_spawn_sim_objects.launch` and `moelk_tables_x3_spawn_sim_objects.launch` take `robot_colours`
(default `false`: everything looks as before). With `robot_colours:=true`:

- the four side covers of robot k show its colour around the unchanged DFKI logo (xacro arg `robot_colour_index` of
  `mobipick_v1_model.urdf.xacro`, passed down by `upload_mobipick_description.launch`,
  `mobipick_gazebo_common.launch`, `mobipick_empty_world.launch` and `spawn_mobipick.launch`; 0 = unpainted, the
  default, which the real robot always uses). The painted cover textures and meshes in
  `mobipick_description/meshes/structure/visual/v2/robot_colours/` are made from the cover texture and the colour
  table by `mobipick_description/scripts/make_robot_colour_covers.py`: run it again after changing either;
- `nodes/spawn_wall_stripe.py` (one per robot, in the robot's namespace) spawns a static, visual-only model
  `<namespace>__wall_stripe`: a band of thin boxes in the robot's colour just in front of every vertical wall face of
  that robot's copy of the world mesh, by default from 0.6 m to 0.2 m below the wall top (moelk: 2.0 to 2.4 m), so
  cameras looking at the tables and the base lasers do not see it. It has no collision; the floor stays as it is.

`spawn_wall_stripe.py` works with any static STL world. Private parameters: `~mesh` (package:// URI, file:// URI or
path, binary or ASCII STL), `~offset_x`/`~offset_y` (where this copy of the mesh is placed), `~z_low`/`~z_high`
(override the band), `~colour` (robot index or namespace, default the node's namespace), `~inset` (0.006 m),
`~thickness` (0.01 m), `~model_name`, `~wait_timeout` (seconds to wait for `/gazebo/spawn_sdf_model`, 0 = no limit).
The geometry (`mobipick_gazebo.wall_stripe`) is tested by `test/test_wall_stripe.py`:

    cd mobipick_gazebo && python3 -m pytest -q test/test_robot_colours.py test/test_wall_stripe.py
