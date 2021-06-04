# mobipick_bringup

This package contains launch and config files to start the most important
systems on the real mobipick robot. After running the launch files from this
repository as described below, you should have the following capabilities set
up:

- Connection to the MIR base, allowing you to drive around using `move_base`
- Drivers for the UR5, the gripper, the force torque sensor, the Astra camera
  and the pico flexx camera on the gripper are running
- A `move_group` is running and configured correctly. You should be able to
  plan and execute trajectories through rviz motion planning plugin.

This package does not provide any other demo capabilities. For high level
planning, environment representation etc, take a look at `pbr_mobipick_demo`
from which you can run the `mobipick_ico_real.launch` _instead_ of
`mobipick_bringup_both.launch`, and `autonomy.launch` to run environment
representation, task planning and object recognition.

## Usage

**mobipick_bringup.launch** needs to be started on the `mobipick-os-sensor` pc.
It loads the URDF model of the robot, creates the connection to the MIR base
and starts camera drivers.

Using the `namespace` and `tf_prefix` arguments you can select the prefixes for
all nodes and tf frames. TF frames are changed without using the
`tf_prefix`-rosparam (since it is deprecated), but by passing its value to
xacro, to prefix the robot joints in the `robot_description`, and to several
config files.

**mobipick_bringup_control.launch** needs to be started on the
`mobipick-os-control` pc. It creates the connection to the UR5, the robotiq
gripper and force torque sensor. Both the grippers and the force torque sensors
serial cables are led into the UR control box, where their are connected to the
power supply, but their data lines are converted to USB there and must be
connected to the `mobipick-os-control` pc. Make sure that the matching of
device to usb2serial-converter is not messed up, as the udev-rules on the
control pc detect the converters, not the devices. Else you might end up
addressing the gripper as `/dev/ft_sensor` instead of `/dev/gripper`, and vice
versa.

**mobipick_bringup_both.launch** wraps both steps above into one single
launch file. Start this from the mobipick-os-sensor pc and it will automatically
run mobipick_bringup_control.launch on the mobipick-os-control pc.
