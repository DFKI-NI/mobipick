# mobipick_gripper_effort_controller

ros_control plugin for the simulated Robotiq 2F-140 gripper on Mobipick.

The previous simulation stack (`robotiq_2f_140_command_bridge.py` in front of a
`position_controllers/JointTrajectoryController`) accepted `GripperCommand.max_effort` and then
dropped it: a position controller cannot limit torque. This controller claims the finger joint on
the `EffortJointInterface`, runs a PID on the joint position error and clamps the PID output to the
torque mapped from `max_effort`, so the limit is enforced by Gazebo.

* Action: `control_msgs/GripperCommand` at `<robot ns>/gripper_hw` (configurable with `action_name`),
  the same name and units as the real Robotiq action server: `position` is the jaw gap in metres,
  `max_effort` is on the 30..100 Robotiq scale.
* Goal validation follows `robotiq_2f_gripper_action_server`: `position` must be within
  `min_gap..max_gap` (-0.015..0.14 m, the values `mobipick_bringup_control.launch` passes to the real
  server) and `max_effort` within `min_effort..max_effort` (30..100). Anything else, including
  `max_effort: 0`, is refused with the real server's warning and the gripper does not move. The real
  server accepts such a goal and then silently never finishes it; the simulation rejects it instead so
  clients get a terminal state. Code that works against this controller therefore also works on the
  real robot, but not necessarily the other way round.
* A stalled gripper (blocked by an object) finishes the goal as succeeded with `stalled: true`,
  like the real server does when the Robotiq reports an object (`gOBJ`); this is what MoveIt's pick
  pipeline and grasplan expect. Set `stall_is_success: false` to abort instead.
* The position setpoint is ramped at `max_joint_velocity` so free motion is smooth and the torque
  limit only becomes active when the fingers are blocked. `goal_timeout` guarantees a goal never hangs.
* State on `<controller ns>/state` (`control_msgs/JointControllerState`; `command` is the applied
  torque, `p`/`i_clamp` carry the active torque limit for plotting).

Parameters and defaults are documented in `config/robotiq_2f_140_gripper_effort_controller.yaml`.
The URDF transmission of the finger joint must use `hardware_interface/EffortJointInterface`.

Tests: `catkin run_tests mobipick_gripper_effort_controller` (gtest on the ROS-free mapping helpers).
