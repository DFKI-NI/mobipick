#!/usr/bin/env python3
"""Render a video of the cable model on the UR5 for recorded joint states (no ROS master needed).

  render_cable_video.py --urdf robot.urdf --event events/<stop>.json.gz --out stop.mp4
  render_cable_video.py --urdf robot.urdf --bag teach.bag --start 20 --end 80 --out teach.mp4
  render_cable_video.py --urdf robot.urdf --csv teach.bag.arm_path.csv --out teach.mp4

The arm is drawn from the model's own collision shapes (config/cable_model.yaml), the carrier of every span as the
carrier link chain (links under gravity with a disc, two crossed diameters, on every link, coloured per span: the
part
of the model that remembers where the loop hangs) plus the geodesic chord between the clamps, so the video shows
exactly what the model computes. Two views per frame; the title carries the time, the joint angles, the stretch per
span (geometric / chain) and the chain's strain, and the decision level. Made to compare the model with videos of
the real robot. It needs the C++ solver (native/disc_chain.cpp; disc_native.py compiles it with g++ when
no built library is found). (The radius rope it drew until 2026-09-30 was removed.)

Two-panel video (real | discs) of Oscar's phone video of 2026-09-27 (sync: video t = bag t + 8.0 s; the phone video
is portrait, stored rotated, ffmpeg turns it upright by itself). Heavy job: nice, OMP_NUM_THREADS=2.

  D=big_outputs/0142_video_sync
  nice env OMP_NUM_THREADS=2 render_cable_video.py --urdf robot.urdf --csv $D/video_sync_*.bag.arm_path.csv \
    --views "0,-80" --figsize 5.4,9.6 --zoom 1.5 --out model_disc_link.mp4
  ffmpeg -ss 8 -t 113 -i $D/IMG_4114.MOV -i model_disc_link.mp4 -filter_complex \
    "[0:v]scale=540:960,fps=15[a];[a][1:v]hstack=inputs=2" -pix_fmt yuv420p two_panel.mp4
"""
import argparse
import gzip
import json
import math
import os
import subprocess
import sys
import tempfile

import numpy as np
import yaml
from mpl_toolkits.mplot3d.art3d import Line3DCollection

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from mobipick_cable_entanglement.cable_model import ARM_JOINTS, CableModel, config_from_dict  # noqa: E402

DEFAULT_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "cable_model.yaml")
SPAN_COLORS = {"wrist": "#d62728", "elbow": "#1f77b4", "shoulder": "#2ca02c"}


def load_urdf(path):
    text = open(path).read()
    if not text.lstrip().startswith("<"):
        text = yaml.safe_load(text)  # rosparam get output is YAML-quoted
    return text


def samples_from_event(path):
    """(t, q) list from a monitor event file (joint_samples, joint order ARM_JOINTS), t relative to the event"""
    event = json.load(gzip.open(path, "rt"))
    order = event.get("joint_order", ARM_JOINTS)
    out = []
    for s in event["joint_samples"]:
        q = dict(zip(order, s["q"]))
        out.append((s["t"] - event["t"], [q[j] for j in ARM_JOINTS]))
    return out, f'{event.get("event", "event")} {event.get("time_utc", "")[:19]}Z'


def samples_from_csv(path):
    """(t, q) list from a CSV with columns t, the six UR5 joints in degrees[, ...] (temp_code/0137_teach_analysis)"""
    import csv

    rows = list(csv.reader(open(path)))[1:]
    return [(float(r[0]), [math.radians(float(v)) for v in r[1:7]]) for r in rows], os.path.basename(path)


def samples_from_bag(path, topic):
    import rosbag  # only needed for bags

    out, t0 = [], None
    with rosbag.Bag(path) as bag:
        for _, msg, stamp in bag.read_messages(topics=[topic]):
            names = [n.split("/")[-1].replace("ur5_", "").replace("_joint", "") for n in msg.name]
            if not all(j in names for j in ARM_JOINTS):
                continue
            t = (msg.header.stamp.to_sec() or stamp.to_sec())
            t0 = t if t0 is None else t0
            out.append((t - t0, [msg.position[names.index(j)] for j in ARM_JOINTS]))
    return out, os.path.basename(path)


def disc_segments(chain, radius):
    """two crossed diameters of the disc on every link of the chain (link k: joint k to k + 1, disc in the middle,
    across the link): a cheap way to draw a hundred discs per frame"""
    links = chain[1:] - chain[:-1]
    length = np.linalg.norm(links, axis=1, keepdims=True)
    t = links / np.maximum(length, 1e-9)
    ref = np.where(np.abs(t[:, 2:3]) < 0.9, np.array([[0.0, 0.0, 1.0]]), np.array([[1.0, 0.0, 0.0]]))
    u = np.cross(t, ref)
    u /= np.maximum(np.linalg.norm(u, axis=1, keepdims=True), 1e-9)
    v = np.cross(t, u)
    mid = (chain[1:] + chain[:-1]) / 2.0
    return [np.stack([m + radius * d, m - radius * d]) for m, a, b in zip(mid, u, v) for d in (a, b)]


def draw(ax, model, q, state, elev, azim, box, zoom=1.0):
    """one view: shapes of arm and tool (grey), carrier link chain per span (coloured), clamps (black), chord
    (dashed)"""
    anchors, caps = model.anchors(q)
    ax.cla()
    center, half = box
    edges = []
    for a, b, r, core_radius, box in caps:
        radius = r - model.cfg.cable_carrier_radius   # the model widens every shape by the carrier radius
        if box[0] > 0.0:   # rounded box, drawn as a cuboid of its full size
            rot = np.asarray(box[3:]).reshape(3, 3)
            corners = {(i, j, k): np.asarray(a) + rot @ (np.array([i, j, k]) * (np.asarray(box[:3]) + radius))
                       for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)}
            edges += [(corners[c1], corners[c2]) for c1 in corners for c2 in corners
                      if c1 < c2 and sum(x != y for x, y in zip(c1, c2)) == 1]
        elif core_radius > 0.0:   # flat-ended cylinder, drawn as two circles at its faces and four lines
            a, b = np.asarray(a), np.asarray(b)
            u = (b - a) / max(np.linalg.norm(b - a), 1e-9)
            e1 = np.cross(u, [1.0, 0.0, 0.0] if abs(u[0]) < 0.9 else [0.0, 1.0, 0.0])
            e1 /= np.linalg.norm(e1)
            e2 = np.cross(u, e1)
            rr = core_radius + radius
            ring = [rr * (math.cos(t) * e1 + math.sin(t) * e2) for t in np.linspace(0, 2 * math.pi, 25)]
            for end in (a - radius * u, b + radius * u):
                edges += [(end + p, end + q) for p, q in zip(ring[:-1], ring[1:])]
            edges += [(a - radius * u + ring[i], b + radius * u + ring[i]) for i in range(0, 24, 6)]
        else:   # capsule: thick grey segment, width ~ radius
            ax.plot([a[0], b[0]], [a[1], b[1]], [a[2], b[2]], color="0.5",
                    linewidth=max(2.0, r * 520 / half * 0.35), alpha=0.5, solid_capstyle="round")
    if edges:
        ax.add_collection3d(Line3DCollection(edges, colors="0.45", linewidths=0.6, alpha=0.7))
    discs = model.disc_points()
    disc_radius = model.cfg.cable_carrier.link_chain.disc_diameter / 2.0
    for name, chain in discs.items():
        color = SPAN_COLORS.get(name, "k")
        ax.plot(chain[:, 0], chain[:, 1], chain[:, 2], color=color, linewidth=1.5)
        ax.add_collection3d(Line3DCollection(disc_segments(chain, disc_radius), colors=color, linewidths=0.8))
    for name, (a, _, b, _) in anchors.items():
        ax.plot([a[0], b[0]], [a[1], b[1]], [a[2], b[2]], color=SPAN_COLORS.get(name, "k"), linewidth=0.8,
                linestyle="--", alpha=0.6)
        ax.scatter([a[0], b[0]], [a[1], b[1]], [a[2], b[2]], color="k", s=12)
    ax.quiver(center[0] - half * 0.85, center[1] - half * 0.85, center[2] + half * 0.8, 0, 0, -half * 0.3,
              color="k", arrow_length_ratio=0.3, linewidth=1.0)  # gravity
    ax.set_xlim(center[0] - half, center[0] + half)
    ax.set_ylim(center[1] - half, center[1] + half)
    ax.set_zlim(center[2] - half, center[2] + half)
    ax.set_box_aspect((1, 1, 1), zoom=zoom)
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    ax.set_title(f"view elev {elev:.0f} azim {azim:.0f}", fontsize=8)


def bounding_box(model, samples, margin=0.12):
    """fixed view box around the arm over the whole clip: (center, half size) in the arm base frame [m]"""
    pts = []
    for _, qv in samples[:: max(1, len(samples) // 50)]:
        _, caps = model.anchors(dict(zip(ARM_JOINTS, qv)))
        for a, b, r, *_ in caps:
            pts.extend([a, b])
    pts = np.array(pts)
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    return (lo + hi) / 2.0, max(hi - lo) / 2.0 + margin


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--urdf", required=True, help="robot_description saved to a file (rosparam get output is fine)")
    p.add_argument("--config", default=DEFAULT_CONFIG)
    p.add_argument("--event", help="monitor event file (*.json.gz) with joint_samples")
    p.add_argument("--bag", help="rosbag with joint states")
    p.add_argument("--csv", help="CSV: t, six UR5 joint angles in degrees")
    p.add_argument("--topic", default="/mobipick/joint_states")
    p.add_argument("--start", type=float, default=-1e9, help="first second to render (event: relative to the stop)")
    p.add_argument("--end", type=float, default=1e9)
    p.add_argument("--fps", type=float, default=15.0)
    p.add_argument("--speed", type=float, default=1.0, help="playback speed (2 = twice as fast as real time)")
    p.add_argument("--views", default="20,-60;20,30",
                   help="elev,azim per view [deg], ';'-separated; Oscar's phone video of 2026-09-27: 0,-80")
    p.add_argument("--figsize", default="12.8,7.2", help="width,height of a frame [inch at 100 dpi]")
    p.add_argument("--zoom", type=float, default=1.0, help="3-D zoom of each view (1.5 fills a portrait pane)")
    p.add_argument("--margin", type=float, default=0.12, help="space around the arm's reach over the clip [m]")
    p.add_argument("--out", required=True)
    args = p.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    cfg = config_from_dict(yaml.safe_load(open(args.config)))
    model = CableModel(load_urdf(args.urdf), cfg)
    if args.event:
        samples, label = samples_from_event(args.event)
    elif args.csv:
        samples, label = samples_from_csv(args.csv)
    else:
        samples, label = samples_from_bag(args.bag, args.topic)
    if not samples:
        sys.exit("no joint samples")
    views = [tuple(float(v) for v in view.split(",")) for view in args.views.split(";")]

    # the chain runs through every sample from the first one (its history matters), frames only inside the window
    model.reset(dict(zip(ARM_JOINTS, samples[0][1])))
    frame_dt, next_frame, last_t, n = args.speed / args.fps, max(args.start, samples[0][0]), samples[0][0], 0
    tmp = tempfile.mkdtemp(prefix="cable_video_")
    box = bounding_box(model, [s for s in samples if args.start <= s[0] <= args.end] or samples, args.margin)
    fig = plt.figure(figsize=tuple(float(v) for v in args.figsize.split(",")), dpi=100)
    axes = [fig.add_subplot(1, len(views), i + 1, projection="3d") for i in range(len(views))]
    fig.subplots_adjust(left=0.0, right=1.0, bottom=0.03, top=0.88, wspace=0.0)
    narrow = fig.get_figwidth() < 8.0
    sep = "\n" if narrow else "; "
    fig.text(0.01, 0.01, f"MODEL PREDICTION: grey arm/tool collision shapes{sep}carrier spans = carrier link chain "
             f"under gravity (red wrist, blue elbow, green shoulder){sep}dashed = chord between clamps (dots), "
             "arrow = gravity",
             fontsize=8)
    for t, qv in samples:
        q = dict(zip(ARM_JOINTS, qv))
        state = model.update(q, min(0.1, max(1e-3, t - last_t)), with_gradient=False)
        last_t = t
        if t < next_frame or t > args.end:
            continue
        repeats = 1 + int((t - next_frame) // frame_dt)   # sparse samples (e.g. a 10 Hz CSV): hold the frame so
        next_frame += repeats * frame_dt                    # the video keeps real time
        for ax, (elev, azim) in zip(axes, views):
            draw(ax, model, q, state, elev, azim, box, args.zoom)
        spans = ("\n" if narrow else "  ").join(f"{k} {s.l_geo / s.free_length - 1:+.0%}/{(s.l_disc or 0) / s.free_length - 1:+.0%}"
                          f"{' HOOKED ' + ','.join(s.hooked_on) if s.hooked_on else ''}" for k, s in state.spans.items())
        spans += ("\n" if narrow else "  ") + "chain strain: " + " ".join(
            f"{k} {s.disc_strain:+.1%}" for k, s in state.spans.items() if s.disc_strain is not None)
        joints = f"q [{', '.join(f'{math.degrees(v):.0f}' for v in qv)}] deg"
        fig.suptitle(f"{label}\nt {t:+6.1f} s level {state.level}\n{joints}\nstretch geo/chain: {spans}" if narrow else
                     f"{label} t {t:+6.1f} s level {state.level}\n{joints} stretch geo/chain: {spans}", fontsize=10)
        fig.savefig(os.path.join(tmp, f"f{n:05d}.png"))
        for k in range(1, repeats):
            os.link(os.path.join(tmp, f"f{n:05d}.png"), os.path.join(tmp, f"f{n + k:05d}.png"))
        n += repeats
    plt.close(fig)
    if n == 0:
        sys.exit("no frame in the requested window")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(args.fps), "-i",
                    os.path.join(tmp, "f%05d.png"), "-pix_fmt", "yuv420p", "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2",
                    args.out], check=True)
    print(f"{n} frames -> {args.out}")


if __name__ == "__main__":
    main()
