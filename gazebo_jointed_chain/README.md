# gazebo_jointed_chain

A Gazebo (11) model plugin that hangs a jointed chain between two links of a model, e.g. a cable carrier between two
clamps of a robot arm. Simulation only; nothing here knows a particular robot.

At load the plugin builds the chain as its own model: `links` rigid capsule links of equal length (`length` / `links`),
joined by universal joints (two bend axes with `bend_limit` each, no twist; ODE ball joints cannot be limited) with
`damping`, laid in a circular arc of the free length below the chord between the two clamp points. Its first and last
link are then held at the clamps by joints to `parent_link` and `child_link` of the host model. The chain collides
only with the host collisions listed in `collide_with` (a token found anywhere in the collision's name, since Gazebo
renames the collisions of links it lumps over fixed joints; or `link/collision`): they get the chain's
`collide_bitmask` alone, so they touch nothing but the chain (add them to the host model for that purpose, e.g. the
shapes the cable runs over). Every other collision of the host model and, with `exclusive` (default true), of every
other model in the world (also models spawned later) loses that bit. Links of the chain that are not neighbours
collide with each other.

## Parameters (children of the `<plugin>` element)

| element | default | meaning |
| --- | --- | --- |
| `chain_name` | `<model>_chain` | name of the chain model, prefix of its ROS topic and service |
| `parent_link`, `child_link` | (required) | host links that hold the two ends |
| `parent_offset`, `child_offset` | `0 0 0 0 0 0` | clamp point in that link's frame |
| `links` | 15 | number of rigid links |
| `length` | 0.52 | free length of the chain [m] |
| `radius` | 0.0175 | link radius [m] |
| `mass_per_link` | 0.0122 | [kg] |
| `inertia_scale` | 8 | multiplies the links' inertia (not their mass); light links next to a heavy arm are unstable in ODE's quickstep otherwise (8 with 12 g links, i.e. the inertia of 0.1 kg, was stable at 200 iterations and a 3 ms step) |
| `bend_radius` | (none) | minimum bend radius [m]; the limit per bend axis is link length / bend radius |
| `bend_limit` | 0.35 | limit per bend axis [rad] when no `bend_radius` is given; soft: ODE lets it a few degrees over |
| `exclusive` | true | clear the chain's bit from every other model's collisions (nothing but the listed shapes touches the chain) |
| `damping` | 0.002 | joint damping per axis [Nms/rad]; much more (0.05) made the light chain explode |
| `friction` | 0.3 | Coulomb coefficient of the chain's collisions |
| `collide_bitmask` | `0x8000` | the chain's bit |
| `collide_with` | (none) | host collisions (name tokens, space separated) that collide with the chain only |
| `reference_link` | world | frame of the published points |
| `settle_time` | 2.0 | [s] sim time over which joints that hang bent more than the limit (an initial shape from a tighter model) narrow their widened limit to the bend limit; no stop call meanwhile |
| `reference_offset` | `0 0 0 0 0 0` | the published frame's pose in `reference_link` (for a frame Gazebo merged into it) |
| `initial_points` | (none) | the chain's shape at spawn, `x y z` triples in the parent link frame (e.g. a settled model of the same carrier, so it starts on the right side of every shape), resampled to the links; used while its ends are within 2 cm of the clamps (also by `reset`), else the arc below the chord |
| `robot_namespace` | `""` | ROS namespace of the topic and services |
| `rate` | 10 | publish rate [Hz] of the state |
| `stop_strain`, `stop_force` | 0 (off) | thresholds of the stop call: chain strain, clamp force [N] |
| `stop_ticks` | 3 | published ticks over a threshold before the call |
| `stop_service` | (none) | `std_srvs/SetBool` called with `true` once per excursion over a threshold |

## ROS interface

- `<ns>/<chain_name>/state` (`std_msgs/Float64MultiArray`, `rate` Hz): `[force_a, force_b, strain, n, x0, y0, z0, ...]`:
  the constraint force at the two clamps [N], the strain (length through the joint points / free length - 1; the links
  are rigid, so it grows when the joints are pulled apart by a pull the chain cannot follow), the number `n` of points,
  then the points in `reference_link`: clamp A, the start and end of every link, clamp B.
- `<ns>/<chain_name>/reset` (`std_srvs/Empty`): hangs the chain afresh (next physics step): in the initial shape while it
  still fits the clamps, else in its arc below the chord.

## Test

`test/chain_test.py` against `test/test_world.sdf` (a toy arm with a driven forearm and wrist, an obstacle only the chain
collides with): the chain hangs below the chord, keeps its length over 1000 steps, bends within the (soft) limit,
follows a swing, calls the stop service on a pull, and re-hangs on reset. Run it in a throwaway container with
`coding_agent/temp_code/0263_chain_plugin_test.sh` (builds the package, own roscore and gzserver).

## Spawn: straight first

Gazebo measures joint angles from the shape a model is spawned in, so the bend limits and the bending springs refer
to that shape. The chain is therefore spawned straight (20 m below the world origin), moved into its hanging shape (the
arc below the chord or `initial_points`) when it is attached, and only then clamped: its springs pull towards a
straight carrier. Spawned in the arc instead, the arc itself would be stress-free (found 2026-10-01, #263).
