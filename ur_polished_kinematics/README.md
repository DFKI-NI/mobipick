# UR5 polished kinematics

This MoveIt plugin enumerates the nominal UR5 closed-form inverse kinematics branches from ROS-Industrial `ur_kinematics`, then corrects each candidate with the robot's loaded URDF chain using damped least squares. Only corrected solutions within the joint limits and 0.1 mm / 0.0001 rad of the target are returned. Candidates are ordered by distance to the seed. The MoveIt multi-solution API exposes every corrected branch.

The upstream analytic solver and MoveIt adapter come from ROS-Industrial universal_robot (`noetic-devel`, `ur_kinematics`); their BSD source headers and `LICENSE_BSD` are retained. This package supports six-joint UR5 chains only. Select `ur_polished_kinematics/UR5PolishedKinematicsPlugin` in a MoveIt kinematics YAML to use it. The default kinematics configuration remains TRAC-IK.
