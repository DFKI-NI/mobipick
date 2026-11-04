# mobipick_cable_entanglement

Mathematical model of the Mobipick arm cable (the triflex tube with the camera USB and F/T sensor cables) and an
**observe-only** node that decides from the arm joint angles whether the cable is entangled, i.e. over-stretched
so that the UR motors feel extra torque and the robot stops. The math is in [doc/cable_model.md](doc/cable_model.md);
 Its development history is kept outside this repository.

| | simulation | real robot |
| --- | --- | --- |
| detected by | `/use_sim_time` true (or `mode:=sim`) | otherwise (or `mode:=real`) |
| on a predicted entanglement | presses the simulated UR protective stop; after a release that leaves the arm over-stretched, presses it again as soon as the stretch grows (e.g. the dashboard resumed the interrupted trajectory, like the real UR) | nothing (never commands the robot) |
| logging | none (stops there come from this node or the button) | **always**: every tick + an event file per stop / prediction / aborted trajectory |

## The simulated cable carrier: a link chain

The cable carrier is simulated as a chain of links with a disc on every link (C++ solver `native/disc_chain.cpp`, built by
catkin as `libdisc_chain.so` or compiled with g++ on first use). It hangs from its clamps under gravity and remembers
where the loop lies; where it needs more carrier than the geometric check (the shortest string around the housings, from
the pose alone) or is pulled longer than its links allow, it decides. Without its solver the model refuses to start
(monitor and MTC cable guard, a clear error, no fallback); `cable_carrier.enabled: false` runs the geometric check alone. An
error inside one update: that tick the geometric check decides alone, the next update starts the chain again,
hanging freely. Sizes: launch args `disc_height`, `disc_gap`, `disc_diameter` [m] or `cable_carrier.link_chain` in the config.
`geometric_check:=false` (launch arg, ROS parameter `cable_model/geometric_check`): the carrier link chain alone decides.
A reset starts the chain at home as on Oscar's photo and leads it to the pose (doc section 2.2).
RViz shows the discs (`cable_discs`, `cable_links`). Twist is logged only. The radius rope (a free point-mass chain
that ran next to the discs, and alone by default) was removed on 2026-09-30: on the real stops of 2026-09-27/28 it
caught none.

**Names**: the arm's cable guide is the **cable carrier** (it was called tube, rope or disc
chain); a span is one section of it between two clamps, the cables run inside, and the model's chain of disc links is
the carrier link chain. Config keys: `cable_carrier:` (was `rope:`) with `link_chain:` (was `disc_link:`),
`cable_carrier_radius` (was `tube_radius`), per span `carrier_start` / `carrier_end` / `carrier_free_length` /
`carrier_rest_side` (were `rope_*`), `wrist_anchor: carrier_tip` (was `tube_tip`); Python `cfg.cable_carrier`,
`path_check.carrier_replay`, `CableModel.update(use_carrier=)`. The old names still load and warn once each (a
config from before, the real robot's copy included); they go once every config is migrated. Kept on purpose:
the URDF/SRDF link and joint names (`cable_guide_*_tube_link`, the robot description), the disc sizes
(`disc_height`, `disc_gap`, `disc_diameter`: the links are discs), the native solver (`native/disc_chain.cpp`,
`libdisc_chain.so`) and the real-robot log keys and state fields (`disc`, `disc_hooked`, `l_disc`, `disc_strain`).
Read the findings in [doc/cable_model.md](doc/cable_model.md) section 5.

What the committed config sets (details in the doc):

- **Spans**: only the wrist span is computed by default (every real stop was there); the elbow and shoulder spans
  are kept but off.
- **Wrist span**: the chain is 0.52 m long (`carrier_free_length`) and runs from the support edge of the last forearm
  bracket (z −0.095 on the forearm guide tube) to the terminal element on wrist_3. The geometric check keeps its own
  start and length. The real tube's length is still to be measured.
- **Shapes** (`capsules`): flat cylinders and boxes fitted to the URDF meshes of arm, F/T sensor, gripper and
  camera; `tool: true` marks the end effector parts where a held loop counts as caught. `wrist_3_cable_clamp_gap_fill` fills the 27 mm slot between the wrist_3 cable clamp bar and
  the camera mount plate: a 35 mm tube cannot enter it, but the solver pushed the chain in from both sides and
  friction held it there, which kept the monitor alarmed for hours on the real days.
- **Start-pose cache** (`cable_carrier.link_chain.reset_cache_mrad`, ROS parameter `cable_model/reset_cache_mrad`, 0 = off,
  a careful flag): a reset at a start pose seen before reuses the chain settled there, so the MTC cable guard skips
  most of the settle on repeated start poses.
- **When the monitor's chain starts afresh** (`chain_reset.py`, monitor launch args and ROS parameters
  `cable_model/release_after_rest_s` and `cable_model/reset_at_trajectory_start`, both off, careful flags, oscar
  2026-10-01 ): release after rest (option R, 10 s = the interim policy): when the arm moves again after
  resting that long, the chain starts afresh at the rest pose; reset at trajectory start (the shared trajectory
  initialization): when an executed trajectory starts (grasplan announces each executed MTC solution and guarded
  named move on `cable_model/trajectory_start`; every executing move_group goal counts too), the chain starts afresh
  through `path_check.start_chain`, the same start as the MTC guard's replay of that trajectory. Every reset is
  logged with pose and time. `cable_model/release_during_rest` (off): the rest release happens once the arm has
  rested that long, still resting, instead of at the next motion; with the trajectory reset this is the candidate of
  2026-10-01.

## Run

```bash
roslaunch mobipick_cable_entanglement cable_entanglement_monitor.launch            # mode auto
roslaunch mobipick_cable_entanglement cable_entanglement_monitor.launch sim_trigger_protective_stop:=false
```

Needs `/mobipick/robot_description` and `/mobipick/joint_states`. The tables demo starts it with the launch arg
`cable_monitor:=true` (`tables_demo_bringup` `demo_sim.launch` and `bringup.launch`, default false; the Mobipick Labs
GUI passes true by default). The MTC cable guard in grasplan (`mtc_pick_place.CableGuard`) builds the same model from
this config and replays each planned path with a freshly settled chain (`path_check.py`).

Outputs (namespace `/mobipick/cable_entanglement_monitor/`):

- `state` (`std_msgs/String`, JSON, every tick): level, reason, stretch/required length/winding per span, extra
  joint torques, joint angles, safety mode
- `level` (`std_msgs/String`, latched): `ok`, `warning`, `entangled`
- `max_stretch` (`std_msgs/Float32`)
- `markers` (`visualization_msgs/MarkerArray`, frame `mobipick/ur5_base_link`): the simulated tube per span in
  green while the span is ok, orange when it warns and red when it is entangled, with its stretch as a label
  coloured green/orange/red by level; with `~show_capsules` (default true)
  also the model's collision capsules of arm and tool (namespace `collision_capsules`, housing radius) and the
  clamps (`clamps`), to compare them with the robot meshes in RViz
- `disc_markers` (`visualization_msgs/MarkerArray`, same frame): only the carrier's discs (namespace `cable_discs`,
  same colours), built only while someone listens; the tables demo sim draws them in Gazebo with the package
  gazebo_marker_visuals (seen by gzclient and the experiment camera, not by the robot's cameras). In the sim the
  monitor warns once when nobody listens here `~cable_drawing_check_s` (30, 0 = off) after its start: Gazebo then
  shows no cable, most often because gazebo_marker_visuals was never built (`catkin build gazebo_marker_visuals`)
- services `~reset` (the cable was untangled by hand; also automatic when a protective stop is released) and
  `~mark_event` (real robot: "it just tangled", writes an event file)
- `~untangle_plan` (**sim only**, no motion): which wrist joints would move where to give the cable slack again
- `~chain_state` (`std_srvs/Trigger`, any mode): the model's chain after the last tick as JSON in `message`
  (`CableModel.chain_state()` plus `stamp`, its ROS time, and `reset_at_trajectory_start`): the MTC guard's chain
  replay of a planned path goes on from it instead of a fresh chain (grasplan `~mtc_cable_replay_live_state`), so it
  sees what the motions before the path did to the chain; about 40 KB, built only on request
- `~untangle` (**sim only**, only while the sim protective stop is active; meant for an "untangle cable" button in the
  simulated UR dashboard): moves wrist_3, then wrist_2, then wrist_1 one at a time, each only as far as needed,
  after a MoveIt collision check of the path (`/mobipick/check_state_validity`); it swaps the dashboard's hold
  controller (`arm_velocity_controller`) for `arm_controller` during the motion and back afterwards. The
  protective stop stays active: watch the arm, then release it in the dashboard
- `cable_model/untangle_with_chain` (safe flag, off; read on every untangle): `~untangle_plan` and `~untangle` plan
  from the carried carrier link chain instead of the geometry alone (a stop the chain decided can look slack to the
  geometry, and the chain gets tighter if the wrist turns the wrong way). The plan runs on a second model built from
  the same robot description and configuration, from a snapshot of the live chain: one wrist joint at a time, both
  directions, 2 deg steps, at most `~untangle_chain_max_turn_deg` (90) each way, the nearest angle with slack (the
  chain, plus the geometry while `geometric_check` is on); none: "no slack within 90 deg ...: untangle the cable by hand" and no motion. After such
  an untangle the chain carried along the motion is kept, also at the release that follows (no fresh chain at the
  stop pose, which stopped again on the way back), and during the protective stop a rest does not restart the chain
  (`release_after_rest_s`). The untangle runs in the simulation only: on the real robot (mode `real`) the monitor
  offers neither `~untangle_plan` nor `~untangle` and ignores this flag, so it spends no CPU on an untangle there

## Real robot logs

Default folder: `data/real_robot_logs/` in this package (on the host mount, so it survives the containers; kept
out of git by `data/.gitignore`). UTC names:

- `YYYY-MM-DDZ_cable_monitor.jsonl.gz`: one line per tick (10 Hz): joint angles, level, stretch and lengths per
  span, extra torques, UR safety mode; plus `safety_mode` and `event` lines. About 10 MB per hour.
- `events/YYYY-MM-DD_HH-MM-SSZ_<kind>.json.gz`: 60 s before to 10 s after the event, joint states at up to
  100 Hz with velocities and efforts (motor currents), the model ticks, the disc chain shapes. Kinds:
  `protective_stop`, `predicted_entanglement`, `trajectory_aborted`, `marked_by_operator`.

## Calibrate

```bash
rosparam get /mobipick/robot_description > /tmp/rd.urdf
rosrun mobipick_cable_entanglement evaluate_cable_model.py --urdf /tmp/rd.urdf --logs <log dir>   # real logs
rosrun mobipick_cable_entanglement evaluate_cable_model.py --urdf /tmp/rd.urdf \
    --srdf $(rospack find mobipick_moveit_config)/config/mobipick_os.srdf.xacro --bag some.bag      # poses, bags
```

To see what the model predicts, render a video of it (arm shapes + the tube spans as the disc chain under
gravity) for recorded joint states, e.g. a stop event, a bag or a CSV of joint angles, and compare it with the real
robot (needs matplotlib and ffmpeg):

```bash
rosrun mobipick_cable_entanglement render_cable_video.py --urdf /tmp/rd.urdf \
    --event <log dir>/events/<time>_protective_stop.json.gz --start -20 --end 3 --out stop.mp4
```

Then update the free lengths in `config/cable_model.yaml` and run the tests:
`python3 -m unittest discover -s test` (or `catkin run_tests mobipick_cable_entanglement`).

## Layout

- `src/mobipick_cable_entanglement/`: `kinematics.py` (URDF forward kinematics, numpy only), `geometry.py`
  (cylinder geodesic, string pulling), `cable_model.py` (spans, disc chain, decision); no ROS imports, runs offline
- `src/.../disc_native.py` + `native/disc_chain.cpp`: the disc chain's solver
- `nodes/cable_entanglement_monitor.py`: the ROS node
- `config/cable_model.yaml`: anchors, housings, capsules, free lengths, thresholds
- `scripts/evaluate_cable_model.py`: offline evaluation and calibration
- `src/.../path_check.py`: the replay of a planned joint path through the chain (MTC cable guard); `slack.py`: the
  wrist_3 turn of a grasp that gives the wrist span the most slack (MTC cable guard)
- `scripts/render_cable_video.py`: video of the model's prediction (arm shapes and the disc chain) for recorded joint
  states
- `test/`: unit tests (the sim `robot_description` is a fixture)
