# mobipick_bringup

This package contains launch- and config-files to start the most important systems on the real mobipick robot. After running the launch files from this repository as described below, you should have the following capabilities set up:

- Connection to the MIR base, allowing you to drive around using `move_base`
- Drivers for the UR5, the gripper and the force torque sensor are running
- A `move_group` is running and configured correctly. You should be able to plan and execute trajectories through rviz motion planning plugin.
- [ ] **TODO**: The cameras mounted at the grippers are running and integrated into moveit
- [ ] **TODO**: The pico flexx cameras mounted at the MIR are running and integrated into move base and moveit

This package does not provide any other demo capabilities. For high level planning, environment representation etc, take a look at `pbr_mobipick_demo` from which you can run the `mobipick_ico_real.launch` _instead_ of `mobipick_bringup_both.launch`, and `autonomy.launch` to run environment representation, task planning and object recognition.

## Usage

**mobipick_bringup.launch** needs to be started on the `mobipick-os-sensor` pc. It loads the URDF model of the robot, creates the connection to the MIR base and starts camera drivers.

> _Note: The camera drivers are disabled for now since the physical attachment part is missing in OS._

The robot version can be switched between `hb` (Bremen) ans `os` (Osnabrück) using the `robot_version` argument. Using `namespace` and `tf_prefix` _arguments_ you can select the prefixes for all nodes and tf frames. TF frames are changed without using the `tf_prefix`-rosparam (since not everyone uses it), but by passing its value to xacro, to prefix the robot joints in the `robot_description`, and to several config files.

> _Note: Be aware that the default setting for_ **robot_version** _is_ **hb** _. The prefixes defaults are_ **mobipick** _._

**mobipick_bringup_control.launch** needs to be started on the `mobipick-os-control` pc. It creates the connection to the UR5, the robotiq gripper and force torque sensor. Both the grippers and the force torque sensors serial cables are led into the UR control box, where their are connected to the power supply, but their data lines are converted to USB there and must be connected to the `mobipick-os-control` pc. Make sure that the matching of device to usb2serial-converter is not messed up, as the udev-rules on the control pc detect the converters, not the devices. Else you might end up addressing the gripper as `/dev/ft_sensor` instead of `/dev/gripper`, and vice versa.

**mobipick_bringup_both.launch** wraps both steps above into one single launchfile. Start this from the mobipick-os-sensor pc and it will automatically run mobipick_bringup_control.launch on the mobipick-os-control pc.

> _Note: Since only the os-version of the mobipick uses the mobipick\_bringup_control.launch script (the hb-version runs rock for the control part), the mobipick\_bringup\_both.launch script sets the_ **robot_version** _to_ **os** _._



## Troubleshooting

> ```
> MoveIt! won't work, the node seems to be running but you cannot plan & execute.
> On startup a strange "X Error of failed request [...]" is shown.
> ```

Strangely, when starting moveit in a namespace it behaves differently from when it's started in the root namespace. When you connected to the robot using `ssh -X [...]` the `DISPLAY` variable is set to something like `localhost:11.0`.
Setting `export DISPLAY=:0` fixes this.

> `[ERROR] [...]: Pipeline producer overflowed! <RTPacket>`

The UR5 is not started yet. Press "POWER ON" on the teach in pendant.

> ```
> Moveit complains that no controller is known to control certain joints.
> ```

After running `mobipick_bringup.launch` the `move_group` tries to connect to the controllers run in `mobipick_bringup_control.launch`, and times out after a while. Run  `mobipick_bringup.launch` again.

