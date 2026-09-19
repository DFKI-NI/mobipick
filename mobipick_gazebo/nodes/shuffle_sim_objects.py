#!/usr/bin/env python3
"""Move the Gazebo objects of a Mobipick Labs world to random poses on their tables.

Every world spawns its objects at the same spots, so a dataset recorded from it
(``record_edge_dataset.sh``) sees each object in one place only. This node
re-arranges the layout in the running simulator without restarting it: each
object stays on the table it is on, gets a random position inside the region
its table-mates currently occupy (expanded and clipped to the table top) and a
random rotation about the vertical axis; roll and pitch are kept so objects
lying on their side keep lying. Objects are placed apart from each other using
the cuboid extents of ``config/sim_object_extents.yaml``.

    rosrun mobipick_gazebo shuffle_sim_objects.py --seed 3 --save /data/.../layout.json
    rosrun mobipick_gazebo shuffle_sim_objects.py --restore layout.json   # undo

Table top sizes are taken from ``config/sim_tables.yaml`` (per world, else the
default 1.40 x 0.70 m); the world is ``--world`` or ``$MOBIPICK_WORLD``.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import random
import re
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import rospkg
import rospy
import yaml
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose

PACKAGE_DIR = Path(rospkg.RosPack().get_path("mobipick_gazebo"))
DEFAULT_TABLES = PACKAGE_DIR / "config" / "sim_tables.yaml"
DEFAULT_EXTENTS = PACKAGE_DIR / "config" / "sim_object_extents.yaml"
DEFAULT_EXCLUDE = ("mobipick", "ground_plane", "experiment_camera")
TABLE_RE = re.compile(r"(^|_)table(_|$)")


# ---------------------------------------------------------------------------
# small quaternion helpers (x, y, z, w), no tf dependency
# ---------------------------------------------------------------------------

def quat_mul(a: Sequence[float], b: Sequence[float]) -> Tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_about_z(angle: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(angle / 2.0), math.cos(angle / 2.0))


def yaw_of(q: Sequence[float]) -> float:
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pose_to_dict(pose: Pose) -> Dict[str, List[float]]:
    p, q = pose.position, pose.orientation
    return {"position": [p.x, p.y, p.z], "orientation": [q.x, q.y, q.z, q.w]}


def dict_to_pose(data: Dict[str, Sequence[float]]) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = data["position"]
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = data["orientation"]
    return pose


# ---------------------------------------------------------------------------
# configuration
# ---------------------------------------------------------------------------

def load_table_sizes(path: Path, world: str) -> Tuple[Dict[str, Tuple[float, float]], Tuple[float, float]]:
    """Per-table (width, depth) of the world plus the default for unlisted tables."""
    default = (1.40, 0.70)
    sizes: Dict[str, Tuple[float, float]] = {}
    if path.exists():
        data = yaml.safe_load(path.read_text()) or {}
        default = tuple(float(v) for v in data.get("default", default))
        for name, size in (data.get("worlds", {}).get(world) or {}).items():
            sizes[name] = (float(size[0]), float(size[1]))
    elif world:
        rospy.logwarn("table config %s not found; every table is %.2f x %.2f m", path, *default)
    return sizes, default


def load_radii(path: Path) -> Dict[str, float]:
    """Half of the largest extent per label: a conservative footprint radius for any orientation."""
    if not path.exists():
        rospy.logwarn("object extents %s not found; using 0.12 m for every object", path)
        return {}
    data = yaml.safe_load(path.read_text()) or {}
    return {label: 0.5 * max(float(v) for v in spec["size"]) for label, spec in data.get("objects", {}).items()}


def label_of(model_name: str) -> str:
    return re.sub(r"_\d+$", "", model_name)


# ---------------------------------------------------------------------------
# layout sampling
# ---------------------------------------------------------------------------

class Table:
    def __init__(self, name: str, pose: Pose, size: Tuple[float, float]):
        self.name = name
        self.x, self.y = pose.position.x, pose.position.y
        self.yaw = yaw_of((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
        self.half = (size[0] / 2.0, size[1] / 2.0)
        self.objects: List[str] = []

    def to_local(self, x: float, y: float) -> Tuple[float, float]:
        dx, dy = x - self.x, y - self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c * dx + s * dy, -s * dx + c * dy

    def to_world(self, lx: float, ly: float) -> Tuple[float, float]:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x + c * lx - s * ly, self.y + s * lx + c * ly

    def contains(self, x: float, y: float, margin: float = 0.0) -> bool:
        lx, ly = self.to_local(x, y)
        return abs(lx) <= self.half[0] + margin and abs(ly) <= self.half[1] + margin


def wait_for_model_states(timeout: float = 10.0) -> Dict[str, Pose]:
    msg = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=timeout)
    return dict(zip(msg.name, msg.pose))


def find_tables(states: Dict[str, Pose], sizes: Dict[str, Tuple[float, float]], default: Tuple[float, float]) -> List[Table]:
    return [Table(name, pose, sizes.get(name, default)) for name, pose in states.items() if TABLE_RE.search(name)]


def assign_objects(states: Dict[str, Pose], tables: List[Table], exclude: Sequence[str]) -> Dict[str, Table]:
    """Objects are the non-excluded, non-table models standing inside a table footprint."""
    owner: Dict[str, Table] = {}
    for name, pose in states.items():
        if name in exclude or TABLE_RE.search(name):
            continue
        for table in tables:
            if table.contains(pose.position.x, pose.position.y, margin=0.05):
                owner[name] = table
                table.objects.append(name)
                break
    return owner


def sample_layout(states: Dict[str, Pose], owner: Dict[str, Table], radii: Dict[str, float], rng: random.Random,
                  expand: Tuple[float, float], edge_margin: float, gap: float, tries: int,
                  default_radius: float) -> Dict[str, Pose]:
    """New pose per object; an object that finds no free spot keeps its current pose."""
    new_states: Dict[str, Pose] = {}
    for table in {t.name: t for t in owner.values()}.values():
        local = {name: table.to_local(states[name].position.x, states[name].position.y) for name in table.objects}
        xs, ys = [p[0] for p in local.values()], [p[1] for p in local.values()]
        limit_x, limit_y = table.half[0] - edge_margin, table.half[1] - edge_margin
        lo_x, hi_x = max(min(xs) - expand[0], -limit_x), min(max(xs) + expand[0], limit_x)
        lo_y, hi_y = max(min(ys) - expand[1], -limit_y), min(max(ys) + expand[1], limit_y)
        rospy.loginfo("%s: %d objects, sampling x in [%.2f, %.2f], y in [%.2f, %.2f] (table frame)",
                      table.name, len(table.objects), lo_x, hi_x, lo_y, hi_y)
        placed: List[Tuple[float, float, float]] = []
        # biggest objects first: they are the hardest to fit
        for name in sorted(table.objects, key=lambda n: -radii.get(label_of(n), default_radius)):
            radius = radii.get(label_of(name), default_radius)
            for _ in range(tries):
                lx, ly = rng.uniform(lo_x, hi_x), rng.uniform(lo_y, hi_y)
                if all(math.hypot(lx - px, ly - py) >= radius + pr + gap for px, py, pr in placed):
                    break
            else:
                rospy.logwarn("%s: no free spot for %s after %d tries; it keeps its pose", table.name, name, tries)
                lx, ly = local[name]
            placed.append((lx, ly, radius))
            old = states[name]
            pose = Pose()
            pose.position.x, pose.position.y = table.to_world(lx, ly)
            pose.position.z = old.position.z + 0.02  # drop it onto the table from just above
            q = quat_mul(quat_about_z(rng.uniform(-math.pi, math.pi)),
                         (old.orientation.x, old.orientation.y, old.orientation.z, old.orientation.w))
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
            new_states[name] = pose
    return new_states


def apply_states(poses: Dict[str, Pose], settle: float) -> Dict[str, Pose]:
    rospy.wait_for_service("/gazebo/set_model_state", timeout=10.0)
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    # lift everything first so an object is never dropped onto one that has not moved yet
    for phase in ("lift", "place"):
        for name, pose in poses.items():
            state = ModelState(model_name=name, reference_frame="world")
            state.pose = Pose()
            state.pose.position.x, state.pose.position.y = pose.position.x, pose.position.y
            state.pose.position.z = pose.position.z + (0.5 if phase == "lift" else 0.0)
            state.pose.orientation = pose.orientation
            response = set_state(state)
            if not response.success:
                rospy.logwarn("set_model_state(%s) failed: %s", name, response.status_message)
    time.sleep(settle)
    return wait_for_model_states()


def report(before: Dict[str, Pose], after: Dict[str, Pose], names: Sequence[str]) -> int:
    fallen = 0
    for name in names:
        b, a = before[name].position, after[name].position
        drop = b.z - a.z
        flag = ""
        if drop > 0.15:
            flag = "  <-- fell off the table?"
            fallen += 1
        rospy.loginfo("%-26s (%.2f, %.2f) -> (%.2f, %.2f)  z %.3f -> %.3f%s", name, b.x, b.y, a.x, a.y, b.z, a.z, flag)
    return fallen


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--seed", type=int, default=None, help="random seed (default: time based, printed)")
    parser.add_argument("--world", default=os.environ.get("MOBIPICK_WORLD", ""), help="world name for the table config")
    parser.add_argument("--tables", type=Path, default=DEFAULT_TABLES, help="yaml with table top sizes per world")
    parser.add_argument("--extents", type=Path, default=DEFAULT_EXTENTS, help="sim_object_extents.yaml for spacing")
    parser.add_argument("--exclude", nargs="*", default=list(DEFAULT_EXCLUDE), help="models never moved")
    parser.add_argument("--only", nargs="*", default=None, help="move only these models")
    parser.add_argument("--expand", type=float, nargs=2, default=(0.25, 0.05), metavar=("X", "Y"),
                        help="metres added around the current object region along the table width/depth")
    parser.add_argument("--edge-margin", type=float, default=0.12, help="keep object centres this far from the table edge")
    parser.add_argument("--gap", type=float, default=0.02, help="minimum free space between object footprints")
    parser.add_argument("--default-radius", type=float, default=0.12, help="footprint radius of unknown labels")
    parser.add_argument("--tries", type=int, default=500)
    parser.add_argument("--settle", type=float, default=3.0, help="seconds to let the objects come to rest")
    parser.add_argument("--save", type=Path, default=None, help="write the layout (seed, before/after poses) as JSON")
    parser.add_argument("--restore", type=Path, default=None, help="put the objects back to the 'before' poses of a layout file")
    parser.add_argument("--dry-run", action="store_true", help="sample and print, do not touch the simulator")
    args = parser.parse_args(rospy.myargv(sys.argv)[1:])

    rospy.init_node("shuffle_sim_objects", anonymous=True)
    states = wait_for_model_states()

    if args.restore:
        layout = json.loads(args.restore.read_text())
        poses = {name: dict_to_pose(entry["before"]) for name, entry in layout["objects"].items() if name in states}
        after = apply_states(poses, args.settle)
        report(states, after, sorted(poses))
        return 0

    seed = args.seed if args.seed is not None else int(time.time()) % 100000
    rng = random.Random(seed)
    sizes, default_size = load_table_sizes(args.tables, args.world)
    tables = find_tables(states, sizes, default_size)
    if not tables:
        rospy.logerr("no table models in /gazebo/model_states (%s)", ", ".join(sorted(states)))
        return 1
    owner = assign_objects(states, tables, args.exclude)
    if args.only:
        for table in tables:
            table.objects = [n for n in table.objects if n in args.only]
        owner = {n: t for n, t in owner.items() if n in args.only}
    if not owner:
        rospy.logerr("no objects on any table")
        return 1
    rospy.loginfo("world %r, seed %d, %d objects on %d tables", args.world, seed, len(owner),
                  len({t.name for t in owner.values()}))

    new_poses = sample_layout(states, owner, load_radii(args.extents), rng, tuple(args.expand), args.edge_margin,
                              args.gap, args.tries, args.default_radius)
    if args.dry_run:
        for name, pose in sorted(new_poses.items()):
            rospy.loginfo("%-26s -> (%.2f, %.2f, %.2f) yaw %.2f", name, pose.position.x, pose.position.y,
                          pose.position.z, yaw_of((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)))
        return 0

    after = apply_states(new_poses, args.settle)
    fallen = report(states, after, sorted(new_poses))
    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        args.save.write_text(json.dumps({
            "world": args.world, "seed": seed, "stamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "expand": list(args.expand), "edge_margin": args.edge_margin,
            "objects": {name: {"table": owner[name].name, "before": pose_to_dict(states[name]),
                               "after": pose_to_dict(after[name])} for name in sorted(new_poses)},
        }, indent=2))
        rospy.loginfo("layout written to %s", args.save)
    if fallen:
        rospy.logwarn("%d object(s) dropped more than 15 cm; check the layout", fallen)
    return 0


if __name__ == "__main__":
    sys.exit(main())
