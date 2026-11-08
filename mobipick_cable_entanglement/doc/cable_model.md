# Mathematical model of the Mobipick arm cable carrier

Status 2026-09-26: first model, calibrated only with the data we had (the URDF, the joint-limit
comments, three real picks without a stop, the sim inserts of ). No joint configuration at a real
entanglement has ever been recorded; the node logs exactly that on the real robot from now on, and
`scripts/evaluate_cable_model.py --logs` turns those logs into new free lengths.

Update 2026-09-30: the cable carrier's model is the disc-and-link chain alone (section 2.2); the
radius rope that ran before it was removed.

Update 2026-10-01: only the wrist span is on by default, the gap fill box against the wedge
of the chain at the wrist_3 cable clamp (section 2.2), the start-pose cache, and the chain scored on the real stops
(section 5).

Names: the articulated guide that routes the cables along the arm is called the **cable carrier**
(older texts and the dated findings below say tube, rope or disc chain: tube / rope / disc chain = cable carrier).
Its model is the **carrier link chain** (discs and links). Config keys now: `cable_carrier:` (was `rope:`) with
`link_chain:` (was `disc_link:`), `cable_carrier_radius` (was `tube_radius`), per span `carrier_start`, `carrier_end`,
`carrier_free_length`, `carrier_rest_side` (were `rope_*`), `wrist_anchor: carrier_tip` (was `tube_tip`); the old
names still load and warn once. Kept on purpose: the URDF/SRDF `cable_guide_*_tube_link` names, the disc size names
(`disc_height`, `disc_gap`, `disc_diameter`), `libdisc_chain` / `disc_chain.cpp`, and the real-robot log keys. The
dated findings from section 4.1 on stay in their words; read them with the line above.

## 1. What is modelled

The camera USB cable and the F/T sensor cable run inside one flexible corrugated cable carrier (igus triflex R, size 30,
Ø 34.5 mm, mounted with the igus UR clamps `TR-911-965-xxx`). The URDF (`mobipick_description/urdf/cable_guide/
cable_guide.urdf.xacro`) holds its clamped parts: a 0.3 m guide tube on the upper arm, a 0.3 m guide tube on the forearm, and
on wrist_3 a clamp with a terminal element 0.145 m off the tool axis from which an 0.08 m guide tube points back to the
wrist. Between two clamped parts the cable carrier is free and crosses one or more joints. Each such part is a **span**:

| span | from (carrier end) | to (carrier end) | joints crossed | confidence |
| --- | --- | --- | --- | --- |
| wrist | forearm guide tube, wrist end | wrist_3 guide tube end | wrist_1, wrist_2, wrist_3 | medium: every entanglement so far |
| elbow | upper arm guide tube, elbow end | forearm guide tube, elbow end | elbow | medium |
| shoulder | pedestal exit (guessed from the photo) | upper arm guide tube, shoulder end | shoulder_pan, shoulder_lift | low |

Only the wrist span is computed by default (every real stop was there); elbow and shoulder are kept but off
(`spans.<name>.enabled`, ROS parameters `cable_model/span_<name>`, monitor launch args `wrist_span`, `elbow_span`,
`shoulder_span`).

The cable carrier is nearly inextensible along its length (chain links with a defined bend radius and a torsion stop).
It becomes dangerous when the arm pose needs more cable carrier between two clamps than there is: the cable carrier pulls on the
clamps, the UR joints see an external torque, and the UR controller stops (protective stop). Before that, the
USB cable inside can be pulled out of its socket.

All quantities are in the arm base frame `ur5_base_link`; forward kinematics come from the URDF.

## 2. Required length of a span

For a span with carrier ends $A$ (proximal) and $B$ (distal) and exit directions $t_A, t_B$ (along the cable carrier), the
cable carrier cannot leave a clamp sideways, so we use lead points

$$A' = A + \ell\, t_A,\qquad B' = B + \ell\, t_B,\qquad \ell = 0.03\ \mathrm{m}.$$

The required length is the largest of lower bounds of the true shortest cable carrier path:

$$L_{req}(q, h) = \max\big(L_{geo}(q),\ L_{disc}(q, h),\ L_{free}(1 + \varepsilon_{disc}(q, h))\big),$$

where $L_{geo}$ depends only on the joint angles $q$, and the link chain's caught length $L_{disc}$ and its strain
$\varepsilon_{disc}$ also on the history $h$ of the motion (section 2.2).

### 2.1 Geodesic around the joint housings, $L_{geo}$

Each crossed joint $j$ has a housing: a cylinder of radius $r_j$ (UR5 collision meshes: 0.045 m for wrist_1/2,
0.042 m for wrist_3, 0.058 m for the elbow, 0.065/0.075 m at the shoulder) plus the cable carrier radius $r_t$, around
the joint axis. In the joint frame, $A'$ and $B'$ have cylindrical coordinates $(\rho_A, \theta_A, h_A)$ and
$(\rho_B, \theta_B, h_B)$. The cable carrier cannot pass through the housing, so the angle it has to go around is the
*unwrapped* angle

$$\Delta_j = q_j + \theta_B^{child} - \theta_A^{parent} + 2\pi k_j ,$$

with $\theta_B^{child}$ measured in the child link (it turns with the joint), $\theta_A^{parent}$ in the parent
link, and $q_j$ the absolute joint angle the UR reports (it is multi-turn: a wrist turned by a full turn more
gives a winding 360° larger). The integer $k_j$ is fixed once: at the joint's neutral angle $q_j^0$ the cable carrier takes
the short way around, $\Delta_j(q^0) \in (-\pi, \pi]$. Neutral angles: wrist_3 165.7° ($\pi - 0.25$, "where the
cables are least stretched", `joint_limits.yaml`), wrist_2 0°, wrist_1 −90°, elbow 0° (straight arm),
shoulder_pan 0°, shoulder_lift −90°.

The shortest string from $A'$ to $B'$ around a cylinder of radius $R = r_j + r_t$ is:

- no contact, $|\Delta_j| \le \alpha_A + \alpha_B$ with $\alpha = \arccos(R/\rho)$:
  $L_j = \sqrt{\rho_A^2 + \rho_B^2 - 2\rho_A\rho_B\cos\Delta_j + (h_B - h_A)^2}$ (the straight line);
- contact: tangent, helix over the wrap angle $w = |\Delta_j| - \alpha_A - \alpha_B$, tangent. Unrolling the
  cylinder turns this into a straight line, so
  $$L_j = \sqrt{\Big(\sqrt{\rho_A^2 - R^2} + R\,w + \sqrt{\rho_B^2 - R^2}\Big)^2 + (h_B - h_A)^2}.$$

A housing is finite: it holds the cable carrier only when both $h_A$ and $h_B$ lie within its extent along the axis
(± 0.02 m); otherwise the cable carrier slips over its end and the joint is skipped. Then

$$L_{geo} = 2\ell + \max\big(|B' - A'|,\ \max_j L_j\big).$$

### 2.2 Link chain of the cable carrier, with history

Some entanglements depend on where the cable carrier's loop is, not only on the pose: "picks tangle often without the two
untangle poses first". A loop that swung behind the gripper or the camera stays caught there, and the next motion
pulls on it. The real guide is a chain of links (igus triflex R) that turn against each other up to a hard stop.
Oscar's idea: model each free span as rigid links on ball joints, with a disc around every link; neighbouring discs
touch at the largest bend angle. This chain is the only chain model: the radius rope (free point masses with a
minimum bend radius) that ran next to it, and alone by default, was removed on 2026-09-30: over the real stops of
2026-09-27/28 it caught none, its only own signal ("caught on the tool") gave the false alarms, and it hung about
20 % longer than its length. Without its C++ solver the model refuses to start (monitor and MTC cable
guard: a clear error, no fallback); `cable_carrier.enabled: false` runs $L_{geo}$ alone. An error inside one update: that
tick $L_{geo}$ decides alone and the next update starts the chain again, hanging freely. The config keys still say
`cable_carrier` (`cable_carrier:`, `carrier_start`, `carrier_free_length`, ...): they are the chain's.

**Start (untangled)**: the chain starts at the reference pose (home), where Oscar's photo shows the untangled cable carrier
(leaving the forearm bracket and hanging in front of the wrist_1 cap), and is led to the pose along the straight joint
path (`start_at_reference`, steps of `reset_sweep_deg`), the "untangled way there" the twist also assumes. Where that
way pulls it taut (home -> a place pose: +18 %), it starts at the pose itself if that is less taut. At home it starts
as a parabola sagging towards the wrist_1 cap side (`carrier_rest_side`) and settles **slowly** (`settle_keep`: most of
its velocity kept per substep only while it settles): dropped at full speed it swung over the top of the cap and
stayed there (+16.5 mm along the wrist_1 axis; the photo's band -38 to -78 mm); settled slowly it rests at -41 mm.
Measured 2026-09-30: a start at the spawn pose, at observe or at transport, then home: -41 mm, strain below 0.3 %
(started at the pose itself, observe -> home was pulled to +9.3 %). This happens at the node start, after every
released protective stop (a person untangled it), on `~reset` and at the start of every replay of a planned path
(MTC guard). A reset takes 0.6-2.6 s. With grasplan's `~mtc_cable_replay_live_state` the guard's replay starts from
the monitor's live chain instead (`CableModel.chain_state` / `restore_chain_state`, the monitor's `~chain_state`
service): a fresh chain forgets what the motions before the path did, so after a motion that caught or pulled the
chain the replay was optimistic (a drill pick: replay +1.2 %, monitor +2.8 %). The live state is used only when it is
under 1 s old, taken within 0.02 rad of the path's first joint positions and from a model with the same config and
robot description (`CableModel.fingerprint`), and not while the monitor starts its chain afresh at every trajectory
start (`reset_at_trajectory_start`): that monitor does what the fresh replay does. Otherwise the replay starts fresh
and says why.

**Spans of the chain**: the chain runs between the support edges of the clamps, not the tips of the URDF
guide tubes, and ends in the terminal element on wrist_3. Wrist: from the edge of the last forearm bracket (z −0.095)
with 0.52 m of cable carrier; elbow 0.472 m, shoulder
0.5435 m. The geometric check keeps its own start and length (section 4.1).

**Gap fill**: between `wrist_3_cable_clamp_2` and the camera mount plate there is a 27 mm slot.
The real cable carrier (34.5 mm) cannot enter it, but the solver pushed chain nodes in from both sides and friction held
them there: on the real days of 2026-09-27/28 the chain stayed wedged and taut for hours, also at home
(a dump of the stuck chain; friction 0 freed it). The box `wrist_3_cable_clamp_gap_fill` fills that
slot. The rule: an open slot on the cable carrier's side that is narrower than the cable carrier is filled. Filling all eleven such
slots of the shapes made new traps between the boxes, so only this one is filled.

**Start-pose cache**: a reset at a start pose seen before (every joint rounded to that step) reuses the chain settled
there, and glides it over `reset_cache_glide_steps` steps from the cached pose to the exact one; at most
`reset_cache_size` entries. The MTC cable guard resets once per path, so repeated start poses skip most of the
settle: 3.4 s -> 0.9 s in the sim, 28 % less replay time and the same verdicts on the 629 real paths.

**Geometric check off** (`decision.geometric_check: false`, ROS parameter `cable_model/geometric_check`, monitor
launch arg `geometric_check`): the carrier link chain alone decides in the monitor and in the guard's path replay (Oscar's
test of the chain in isolation); `evaluate()` (the guard's geometric margin stage, the servo guard, untangling)
stays geometric.

**Sizes** (`cable_carrier.link_chain`, approximations by Oscar of 2026-09-30, the real cable carrier is still to be measured): disc
height 0.005 m, gap 0.005 m, diameter 0.035 m. Link pitch $p = h + g$ = 0.01 m; the rims of two neighbouring discs
touch at the joint stop $\varphi = 2\arctan(g/d)$ = 16.3° per joint, which is a bend radius of the centre line of
$p / (2\sin(\varphi/2))$ = 35 mm (catalogue TRC.30.050.0: outer Ø 34.5 mm, bend radius 50 mm, twist about ±10° per
link; its pitch is unverified). (Photos gave 90 mm for the removed rope.)

**Solver** (`native/disc_chain.cpp`, plain C interface, loaded with ctypes by `disc_native.py`; catkin builds
`libdisc_chain.so`, else `g++` compiles it into `~/.cache/mobipick_cable_entanglement/`). Position based
dynamics on the joints (point masses), 100 substeps per model update, 2 passes each:

- link lengths and joint stops are projected joint by joint, forwards and backwards along the chain;
- the joint stop is the distance of a joint from the middle between its two neighbours, which grows with the turn.
  (A distance between every second node, as the removed radius rope had, is nearly blind: $2s\cos(\text{turn}/2)$
  loses only 1 % from 0° to 16°, so a link stretched by 2 % hides a joint at twice its stop.);
- capsules of arm and tool, widened by the disc radius, glide from their last to their new place over the substeps
  and are tapered towards a clamp that sits inside them (`clamp_taper`);
- **friction**: Coulomb friction of the cable carrier on the capsules (`friction` 0.3, a guess), against the moving capsule
  surface, so a housing turning about its own axis (wrist_3, the tool) drags the cable carrier;
- the cable carrier does not pass through itself: joints far enough apart along the chain keep one disc diameter;
- real gravity (9.81 m/s²).

**Length and decision.** The chain is tightened (40 red-black Gauss-Seidel smoothing steps with collisions, so a
loop merely lying on a housing slides off), its remaining contacts are string-pulled (drop every point whose
neighbours see each other past the shapes), and the length of what is left is $L_{disc}$, using only the holds
behind the **tool** (gripper, fingers, camera, F/T sensor), i.e. the loop is caught there; it enters $L_{req}$ only
when such a hold exists. $L_{disc}^{topo}$ uses all holds, including wraps around the housings; logged, not used
(`decide_on_topology` false: with the removed rope it made the three real picks without a stop false alarms). Its links cannot stretch: the **strain** (chain length over nominal length
minus one) counts as that much stretch of its span from `strain_tolerance` 1 % on (`decide_on_strain`), which is
the **taut-chain stop**: `decision.stop_stretch` (+2 %) then predicts the protective stop and `limiting` says
"carrier link chain (pulled taut)" (RViz label "chain taut +N%"). The state has a `disc` entry per span (length, holds,
strain, twist). The monitor names the disc sizes in its start line, logs the disc values per tick on the real robot
(`l` keeps $L_{geo}$ only), and draws links and discs in RViz (`cable_discs`, `cable_links`).

**Twist** is logged only, never decides: the turn of the end clamp against the start clamp about the cable carrier
(`twist_deg`), against links × 10°. It comes from the clamps alone: the twist of the straight carrier
between them (leads included), followed from tick to tick and counted from the reference pose after a reset. It is
continuous in the joint angles (at most the wrist joints' own turn per tick), the same whenever the arm stands still,
and back at its start value after a pose loop. The value carried along the chain's shape (parallel transport) is
still logged as `twist_chain_deg`, for diagnosis only: the carrier link chain has no torsion, so it coils and sways
freely, and that value moved by up to 110° in 0.15 s with the arm standing still (real halt E4); a real tube's
torsion resists that. Neither is checked against the real tube yet.

**Cost**: the C++ solver takes about 7.5 ms per update (numpy: 44 ms with 48 substeps); a whole model update about
14.5 ms on the laptop (2026-09-30, with the removed rope next to it 20.3 ms), 10 Hz monitor.

## 3. Tension, torque, decision

With $L_{free}$ the free cable carrier length of the span:

$$s = \frac{L_{req}}{L_{free}} - 1,\qquad T = EA\ \max(0, s),\qquad
\tau_k = \sum_{spans} T\,\frac{\partial L_{geo}}{\partial q_k}.$$

$s<0$ is slack left, $s \ge 0$ taut. $EA$ = 2000 N lumps the cable carrier, the clamps and the cables; $\tau_k$ follows
from virtual work (the tension resists every motion that lengthens the path, with the lever arm
$\partial L/\partial q_k$, central differences). Decision per tick:

- **entangled** (the UR would stop; in the sim the node presses the protective stop) when $s \ge 0.02$ or
  $|\tau_k| \ge \tau_k^{stop}$ (4 Nm wrists, 10 Nm elbow, 15 Nm shoulder) on 2 consecutive ticks;
- **warning** when $s \ge -0.04$ or the loop is caught on the tool;
- **ok** otherwise.

### Untangling in the simulation (Oscar, 2026-09-27)

When the monitor pressed the sim protective stop, a person can press an "untangle" button (service `~untangle`):
for wrist_3, then wrist_2, then wrist_1, the model searches the angle nearest to the current one (both directions,
within the URDF limits, 2° steps) at which the whole cable has at least 6 % slack (warning level − 2 %), stopping
as soon as that is reached; a joint that cannot reach it goes to its least-stretch angle and the next joint
continues. Shoulder and elbow never move. The joint-by-joint path is collision-checked with MoveIt and executed
slowly; the protective stop stays active until the person releases it.

## 4. Calibration with the data of 2026-09-26

| evidence | what it gives |
| --- | --- |
| `joint_limits.yaml`: wrist_1 ≤ 0° and wrist_3 ≥ 30° "to avoid overstretching the cable guide" | at those edges (home family: wrist_2 −90°) the wrist span is about taut: $L_{geo}$ = 0.39 m and 0.41 m |
| real picks `banana_grasp{2,3,4}.bag`, no stop | wrist span ≤ 0.389 m, elbow ≤ 0.338 m, shoulder ≤ 0.424 m |
| named poses in daily use (home, observe, transport, anygrasp, untangle 1/2, ...) | wrist ≤ 0.36 m, elbow ≤ 0.363 m, shoulder ≤ 0.445 m |
| (sim): keeping the grasp orientation turned wrist_3 by 180–210° per insert, the same inserts that entangled on the real robot | from observe (wrist_3 270°), wrist_3 below ~150° needs > 0.44 m |
|  | stops during inspect, place, insert; no joint angles |

Chosen: $L_{free}$ = **0.43 m** (wrist), 0.40 m (elbow), 0.50 m (shoulder, low confidence). With it, all everyday
poses and all real pick samples are ok (wrist ≥ 9 % slack in the picks), the joint-limit edges of the home family
are just below taut, and turning wrist_3 more than ~120° down from observe is predicted to stop, as in .
The `handover` pose (old tables demo only) is predicted to entangle: a cheap first real-robot check.

### 4.1 Wrist span from the real cable carrier support

The 30 cm forearm guide tube of the URDF is longer than the supported section of the real tube (MoveIt keeps its
clearance with it and it stays as is), so the wrist span's deciding start at its tip (z −0.15) counted 5.5 cm of free
tube as clamped. `support_start` (z −0.095, the edge of the last forearm bracket, where the rope starts) with
`support_free_length` decides instead when the config is built with `wrist_anchor: support`; `auto` (the config's
setting) uses it in the sim and keeps the carrier tip (old name: tube tip) on the real robot until tested there. Refit on the real evidence of
2026-09-27/28 (a scan over the real logs, 8649 normal ticks = every 50th, > 5 s before a stop):

| wrist start, $L_{free}$ | E1 | E2 | E3 | E4 | E5? | T1 | normal ticks ≥ −0.115 | normal max |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| tube tip, 0.43 | −0.129 | −0.085 | −0.193 | −0.157 | −0.359 | −0.294 | 3.5 % | −0.015 |
| support, 0.45 | −0.069 | −0.037 | −0.301 | −0.082 | −0.304 | −0.212 | 5.2 % | −0.038 |
| support, 0.465 | −0.099 | −0.068 | −0.324 | −0.112 | −0.326 | −0.238 | 4.6 % | −0.069 |

One second before the stop E3 and T1 are at −0.065 / −0.067 with 0.45 (tube tip −0.131 / −0.136); at the stop instant
their span is chord-limited, and the guard, which checks whole paths, rejects the way in. Chosen **0.45** (a
refit): the guard margin −0.115 catches all five known taut poses (tube tip: E2 only), E4 with 3 points of margin
(0.465: 0.3 points), for about 1 % instead of 0.4 % of everyday poses above the margin on one everyday tick set (5.2 % /
3.5 % on a second one, which counts long stand-stills many times); the named poses get slacker (observe100cm_right −0.18 →
−0.30, anygrasp −0.19 → −0.31). No normal tick reaches the stop threshold. Widening the wrist_1/wrist_2 extent to −0.20
only changes E3's stop instant (still slack here) and costs 1.1 points: not taken. E5? is not explained by the length,
and the thresholds are unchanged.

## 5. Limits of the model (what the real-robot logs must settle)

- **Findings of 2026-09-30, they change how the rope results are read:**
  1. For point masses on fixed segments, today's bend constraint (distance between every second node) already is a
     ball-joint stop; the disc idea changes numbers, not the principle. New are the fine pitch, a harder stop
     (3.5 cm), the tube not passing through itself, friction and the twist.
  2. **The radius rope hangs about 20 % longer than its length** (wrist ×1.24, elbow ×1.18, shoulder ×1.21 at home,
     at rest): pairwise projection with 2 × 12 passes at dt 0.05 s does not hold the segment lengths against gravity. It does not depend on the bend radius (0.03, 0.05, 0.09 give the same), which is why the replay of 
     with 0.05 and 0.12 changed nothing.
  3. **With links that cannot stretch, the measured 0.48 m wrist tube does not fit around the model's capsules at
     `observe100cm_right`**, a pose the real robot holds all day: the wrist chain ends up 16 % longer than it is
     (elbow and shoulder chains stay below 0.4 %), so `disc_link` predicts a stop there, a false alarm of the
     geometry, not of the bend model. Wrist strain at observe by variant: tube 0.52 m 7.3 %, 0.56 m 1.0 %, 0.60 m
     0.5 %; discs of 1 cm 7.9 %; no self-contact and no padding 2.8 %; no clamp taper 4.8 %; no friction 14.1 %;
     catalogue stop (R 50 mm) 17.1 %; a stop of 45° 14.1 %. The tube length and the width of the capsules
     decide, the bend stop hardly matters. Today's radius rope hid this because it hangs 20 % long.
  4. Solvers tried and rejected: all link lengths of a chain at once (the stops give way at the clamps: kinks of
     50-130°), turning links back onto the stop joint by joint (tears the chain in motion), rebuilding the chain
     link by link from the clamps (misses a clamp by centimetres, ends 3 cm inside a capsule).
  Consequence: `disc_link` cannot decide anything before the wrist tube's length along its centre line and
  the capsules are calibrated; until then it is an instrument: at poses known to be slack on the real robot
  the strain must be near zero.
- Quality of the chain on a loop over the named poses with a 0.60 m wrist tube: strain below 1 % also at 40°/s, no
  capsule penetration above 2 mm, elbow joints within 3° of the stop; one wrist joint up to 9° over it where a
  capsule pushes the chain at a clamp, shoulder joints up to 10° over (coiled, pedestal exit guessed).
- The disc sizes, friction (0.3) and twist per link are guesses; wrist_3 and the four tool capsules are still
  hand-measured (the arm capsules were refit to the meshes on 2026-09-27).
- The disc size launch args (`disc_height`, `disc_gap`, `disc_diameter`) exist on the monitor launch only; they
  are not passed through `demo_sim.launch`, `bringup.launch` or the GUI profile.
- **The chain on the real stops of 2026-09-27/28** (disc chain alone, `geometric_check` false, replay
  of the real logs):
  - MTC cable guard (a freshly settled chain per planned path): rejects 4 of the 5 real stop paths (E3 missed)
    and 44 of 618 paths the real robot moved without a stop (7.1 %); the gap fill does not change this.
  - Monitor (the chain carried through the whole day): false alarms on 31.7 % of the ticks before the gap fill,
    3.3 % with it; the wedged home moments are quiet now. It still alarms at none of the five stops at the
    moment of the halt. Its day-long history drifts away from the real tube even without the wedge: a guard started
    from the monitor's chain instead of a fresh one rejects only 1 of 5 stop paths, at 14.9 % of the safe paths.
  - A chain started fresh after every rest of 10 s (no joint moving more than 0.5°) alarms at 4 of 5 stops at the
    halt with 0.6 % false alarm ticks (2 s rest: 4 of 5, 2.4 %). This is a model choice ("history counts only
    within one motion"), proposed to Oscar, not implemented.
  - : the guard refuses 18 of 118 real moves from transport; 9 of them need more than 0.52 m of tube even as a
    string pulled tight (up to 0.646 m), so the tube length decides there, not the guard.

- $L_{free}$ and the stop thresholds are guesses; the pedestal exit of the shoulder span is not in the URDF.
- Neutral angles of wrist_1/wrist_2 (branch of the winding) are assumptions.
- The carrier link chain's sizes, friction and length are approximations; its torsion is not simulated (twist
  is only logged). Carried through a whole day it got pulled taut and stayed taut (alarm on 32-41 % of the
  ticks), mostly wedged in the slot that the gap fill now closes.
- Housings are cylinders and boxes fitted to the meshes.
- At home the real wrist tube hangs in front of the wrist_1 cap; the disc chain settling on its own ends over the
  top (+16.5 mm along the wrist_1 axis, section 2.2); the free length along the tube's centre line is still to be
  measured.
- (Until the wrist rope ended at the tip of the URDF's 8 cm wrist_3 guide tube, where it touched the camera
  capsule, which the hook test took for a caught loop: 32 of 255 real motions of 2026-09-27 flagged. The chain now
  ends in the terminal element.)
- The camera USB plug may be pulled out before the UR stops: then (camera lost, no stop) is also an
  over-stretch, logged as `trajectory_aborted` or via `~mark_event`.

## 6. Refinement procedure

1. Every real-robot session runs the node (`mode` auto → real); it logs every tick and an event file per stop.
2. The planned calibration: move each wrist joint slowly by hand/teach pendant towards its limit, and some
   combinations (observe with wrist_3 going down, home with wrist_1 going up), until the stop; call
   `~mark_event` when the cable carrier looks taut.
3. `evaluate_cable_model.py --urdf <robot_description> --logs <log_dir>` prints, per span, the largest required
   length without a stop and the smallest at a stop, and the free length between them. Update
   `config/cable_model.yaml`, re-run the tests.
