# mobipick_servo

MoveIt Servo for the Mobipick arm (#200): jog the gripper in Cartesian space (twist) or per joint, streamed to the
existing `arm_controller` (no controller switch; the trajectory controller holds its pose when servo goes quiet), with a
cable guard in between.

```
roslaunch mobipick_servo servo.launch          # sim config; namespace mobipick
rostopic pub -r 20 /mobipick/servo_server/delta_twist_cmds geometry_msgs/TwistStamped \
  '{header: {frame_id: mobipick/base_link}, twist: {linear: {z: -0.05}}}'   # 5 cm/s down, stops 0.2 s after the last
```

Measured (sim retest 2026-09-29, #200): a 2 s jog at 0.05 m/s moves the gripper 2.6-2.7 cm, about half the commanded
speed (offline harness with an instantly following fake arm: 5.4 of 10 cm; assumption: servo adds each step to the
measured joint state, which lags one servo cycle). A MoveIt motion works right after a jog.

- `servo_server` = `servo_node` (moveit_servo 1.1.16's servo_server plus a tf buffer for the planning scene monitor,
  which fills Mobipick's floating virtual joint map -> base_footprint): `config/servo_sim.yaml`: group `arm`, EE `mobipick/gripper_tcp`, commands in
  `mobipick/base_link`, 0.1 m/s / 0.3 rad/s, 50 Hz, collision checking against the planning scene (tables, pose
  selector boxes and the robot itself, from move_group's `monitored_planning_scene` with the octomap removed: distance
  checks against voxels are slow and the sim octomap holds phantom arm voxels; that topic must stay on),
  singularity slow-down 17 / stop 30. Collision slow-down starts 0.005 m (self) / 0.02 m (scene) from contact: with
  0.02 m some Mobipick link pair is always that close and servo crawled at ~5 % speed in every pose. Output:
  `servo_server/command_unguarded`.
- `cable_guard_relay.py`: each target goes on to `arm_controller/command` only when the geometric cable model of
  `mobipick_cable_entanglement` keeps the largest span stretch at or below `~max_stretch` (-0.115, like grasplan's
  MTC guard) or does not make it tighter than now (moving out is always allowed), and wrist_3 stays within
  `~wrist_3_half_window_deg` (170) of the cable carrier's neutral angle. A refused target becomes one hold at the current state
  and a message on `cable_guard_relay/blocked`. The model comes from the mobipick_cable_entanglement package; without it nothing is
  forwarded (`cable_guard:=false` relays unchecked). 2-3.4 ms per check. Sim: a wrist_3 jog from 218 deg stops at
  about 110 deg with the #211 cable model, the cable monitor stays quiet, jogging back is allowed at once.
  Servo output is forwarded only within `~input_window_s` (0.3 s) of the last twist or joint jog: moveit_servo 1.1.16
  keeps publishing holds after a client stops without a final zero command, and those would replace every MoveIt
  trajectory on the shared `arm_controller`.
- moveit_servo is not in the image yet (Dockerfile line `ros-noetic-moveit-servo` pending an image rebuild); until
  then clean_mobipick_labs_ws/src/moveit_servo holds a sparse checkout of only moveit_ros/moveit_servo at 1.1.16.
- Real robot: not yet. The UR driver's `scaled_pos_joint_traj_controller/command` would take the same output
  (`command_out:=...`); test in the sim first.

Tests: `python3 -m pytest -q test` (the guard with the real cable model; skipped without mobipick_cable_entanglement).
