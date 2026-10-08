#!/usr/bin/env python3
"""Evaluate and calibrate the cable model offline (no ROS master needed).

  evaluate_cable_model.py --urdf robot.urdf --srdf mobipick_os.srdf.xacro          # named poses
  evaluate_cable_model.py --urdf robot.urdf --bag pick.bag [--bag ...]              # replay joint states
  evaluate_cable_model.py --urdf robot.urdf --logs data/real_robot_logs             # real robot logs -> calibration

The URDF is the robot_description parameter saved to a file (rosparam get /mobipick/robot_description). With
--logs, every protective stop event of the real robot is a positive sample and the ticks without a stop are
negatives: per span it prints the largest required length seen without a stop and the smallest at a stop, the
free length between them that the config should use, and how the current config classifies the stops.
"""
import argparse
import glob
import gzip
import json
import math
import os
import re
import sys

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from mobipick_cable_entanglement.cable_model import ARM_JOINTS, CableModel, config_from_dict  # noqa: E402

DEFAULT_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "cable_model.yaml")


def load_urdf(path):
    text = open(path).read()
    if not text.lstrip().startswith("<"):
        text = yaml.safe_load(text)  # rosparam get output is YAML-quoted
    return text


def named_poses(srdf_path):
    s = open(srdf_path).read()
    out = {}
    for m in re.finditer(r'<group_state name="([^"]+)" group="arm">(.*?)</group_state>', s, re.S):
        out[m.group(1)] = {k: float(v) for k, v in re.findall(r'ur5_(\w+)_joint" value="([-\d.eE]+)"', m.group(2))}
    return out


def fmt_state(st):
    return " ".join(f"{n}={100 * s.stretch:+5.1f}%({s.limiting})" for n, s in st.spans.items())


def read_bag(path, topic):
    """Yields (t [s], {joint: position}) from a ROS 1 bag, with rosbag if available, else the rosbags library."""
    try:
        import rosbag
        with rosbag.Bag(path) as bag:
            for _, msg, t in bag.read_messages(topics=[topic]):
                yield t.to_sec(), dict(zip(msg.name, msg.position))
        return
    except ImportError:
        pass
    from pathlib import Path
    from rosbags.highlevel import AnyReader
    with AnyReader([Path(path)]) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        for c, t, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, c.msgtype)
            yield t / 1e9, dict(zip(msg.name, msg.position))


def replay(model, samples, rate=10.0, verbose=True):
    last, counts, worst = None, {}, None
    for t, q in samples:
        if not any("wrist_3" in k for k in q):
            continue
        if last is not None and t - last < 1.0 / rate:
            continue
        st = model.update(q, 0.0 if last is None else t - last)
        last = t
        counts[st.level] = counts.get(st.level, 0) + 1
        if worst is None or st.max_stretch > worst[1].max_stretch:
            worst = (t, st, q)
        if verbose and st.level == "entangled" and counts[st.level] <= 5:
            print(f" t={t:.1f} entangled: {st.reason}")
    return counts, worst


def calibrate(model, cfg, log_dir, guard_s=3.0):
    """Stops are positives; log ticks are negatives unless they are within guard_s before a stop (the cable was
    already pulling then) or while the robot is stopped."""
    stops = []
    for path in sorted(glob.glob(os.path.join(log_dir, "events", "*_protective_stop.json.gz"))):
        with gzip.open(path, "rt") as f:
            ev = json.load(f)
        before = [s for s in ev["joint_samples"] if s["t"] <= ev["t"]]
        if not before:
            continue
        st = model.evaluate(dict(zip(ARM_JOINTS, before[-1]["q"])))
        stops.append((ev["t"], os.path.basename(path), st))
    stop_times = [t for t, _, _ in stops]
    negatives = {s.name: 0.0 for s in cfg.spans}
    for path in sorted(glob.glob(os.path.join(log_dir, "*_cable_monitor.jsonl.gz"))):
        with gzip.open(path, "rt") as f:
            for line in f:
                row = json.loads(line)
                if "q" not in row or row.get("safety") in (3, 5, 12):
                    continue
                if any(0.0 <= t - row["t"] <= guard_s for t in stop_times):
                    continue
                st = model.evaluate(dict(zip(ARM_JOINTS, row["q"])))
                for n, s in st.spans.items():
                    negatives[n] = max(negatives[n], s.l_req)
    print(f"{len(stops)} protective stop events")
    for _, name, st in stops:
        print(f"  {name}: predicted={st.predicted_stop} {fmt_state(st)}")
    for span in cfg.spans:
        at_stop = [st.spans[span.name].l_req for _, _, st in stops if st.worst_span == span.name]
        lo = negatives[span.name]
        hi = min(at_stop) if at_stop else None
        suggestion = f"{(lo + hi) / 2:.3f}" if hi and hi > lo else "n/a (no separating value)"
        print(f"span {span.name}: max L_req without stop {lo:.3f}, min L_req at a stop "
              f"{'-' if hi is None else f'{hi:.3f}'} -> free_length {suggestion} (config {span.free_length:.3f})")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--config", default=DEFAULT_CONFIG)
    ap.add_argument("--srdf")
    ap.add_argument("--bag", action="append", default=[])
    ap.add_argument("--topic", default="/mobipick/joint_states")
    ap.add_argument("--logs")
    ap.add_argument("--no-carrier", action="store_true", help="geometry only (stateless, no carrier link chain)")
    args = ap.parse_args()

    cfg = config_from_dict(yaml.safe_load(open(args.config)))
    if args.no_carrier:
        cfg.cable_carrier.enabled = False
    urdf = load_urdf(args.urdf)

    if args.srdf:
        model = CableModel(urdf, cfg)
        print("named poses (stateless):")
        for name, q in named_poses(args.srdf).items():
            st = model.evaluate(q)
            print(f"  {name:32s} {st.level:9s} {fmt_state(st)}")
    for bag in args.bag:
        model = CableModel(urdf, cfg)
        counts, worst = replay(model, read_bag(bag, args.topic))
        print(f"{bag}: {counts}")
        if worst:
            t, st, q = worst
            qd = {k.split('ur5_')[-1].replace('_joint', ''): round(math.degrees(v)) for k, v in q.items() if 'ur5' in k}
            print(f" worst t={t:.1f}: {fmt_state(st)} at {qd}")
    if args.logs:
        calibrate(CableModel(urdf, cfg), cfg, args.logs)


if __name__ == "__main__":
    main()
