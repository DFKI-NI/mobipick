#!/usr/bin/env python3
"""gazebo_jointed_chain test against gzserver running test/test_world.sdf (with gazebo_ros's API plugin): the chain
hangs and stays together, its length is conserved, its bends stay near the limit, it follows a swing of the arm, the
stop service is called when the wrist pulls it taut, and the reset service re-hangs it. Prints PASS/FAIL per check
and exits non-zero on a failure.
  python3 chain_test.py
"""
import math
import sys
import time

import rospy
from gazebo_msgs.srv import ApplyJointEffort
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty, SetBool, SetBoolResponse

LIMIT_DEG = math.degrees(0.35)
state = {"msg": None, "clock": None, "stops": 0}


def on_stop(req):
    state["stops"] += 1
    return SetBoolResponse(success=True, message="test")


rospy.init_node("chain_test")
rospy.Subscriber("/arm_chain/state", Float64MultiArray, lambda m: state.__setitem__("msg", m), queue_size=1)
rospy.Subscriber("/clock", Clock, lambda m: state.__setitem__("clock", m.clock.to_sec()), queue_size=1)
rospy.Service("/test_stop", SetBool, on_stop)
rospy.wait_for_service("/gazebo/apply_joint_effort", 60)
effort = rospy.ServiceProxy("/gazebo/apply_joint_effort", ApplyJointEffort)
reset = rospy.ServiceProxy("/arm_chain/reset", Empty)
t0 = time.monotonic()
while (state["msg"] is None or state["clock"] is None) and time.monotonic() - t0 < 60:
    time.sleep(0.1)
failures = []


def check(name, ok, detail):
    print(f"{'PASS' if ok else 'FAIL'} {name}: {detail}", flush=True)
    if not ok:
        failures.append(name)


if state["msg"] is None:
    check("state topic", False, "no /arm_chain/state within 60 s")
    sys.exit(1)


def parse(m):
    fa, fb, strain, n = m.data[:4]
    pts = [tuple(m.data[4 + 3 * i:7 + 3 * i]) for i in range(int(n))]
    return fa, fb, strain, pts


def bends(pts):
    """bend between neighbouring links (deg); pts = clamp A, (start, end) per link, clamp B"""
    links = [(pts[1 + 2 * i], pts[2 + 2 * i]) for i in range((len(pts) - 2) // 2)]
    axes = [tuple(b[k] - a[k] for k in range(3)) for a, b in links]
    out = []
    for u, v in zip(axes, axes[1:]):
        c = sum(x * y for x, y in zip(u, v)) / (math.sqrt(sum(x * x for x in u)) * math.sqrt(sum(y * y for y in v)))
        out.append(math.degrees(math.acos(max(-1.0, min(1.0, c)))))
    return out


def watch(seconds):
    """worst strain range, bend, clamp force and the last state over `seconds` of sim time"""
    start = state["clock"]
    strains, worst_bend, worst_force, nan = [], 0.0, 0.0, False
    while state["clock"] - start < seconds:
        fa, fb, strain, pts = parse(state["msg"])
        nan = nan or any(math.isnan(v) for p in pts for v in p)
        strains.append(strain)
        worst_bend = max([worst_bend] + bends(pts))
        worst_force = max(worst_force, fa, fb)
        time.sleep(0.05)
    return min(strains), max(strains), worst_bend, worst_force, nan, parse(state["msg"])


rospy.sleep(3.0)   # the plugin's limit ramp (settle_time 2 s after the chain is attached) is not the rest
lo, hi, bend, force, nan, (fa, fb, strain, pts) = watch(3.0)   # ~1000 steps at 3 ms
check("hangs below the chord", min(p[2] for p in pts) < pts[0][2] - 0.05, f"lowest point {min(p[2] for p in pts):.3f}")
if rospy.get_param("/chain_test/rest_above", None) is not None:   # a shape under the chain (test_world_host)
    above = rospy.get_param("/chain_test/rest_above")
    check("rests on the shape", min(p[2] for p in pts) > above, f"lowest point {min(p[2] for p in pts):.3f} > {above}")
check("length conserved at rest (1000 steps)", hi - lo < 0.01 and abs(hi) < 0.01, f"strain {lo:+.4f}..{hi:+.4f}")
# the limit is per bend axis (two axes combine to sqrt(2) times it) and soft (ODE): about 5 deg over at rest
check("bends near the limit at rest", bend < LIMIT_DEG * math.sqrt(2) + 6.0,
      f"largest {bend:.1f} deg (limit {LIMIT_DEG:.1f} per axis, {LIMIT_DEG * math.sqrt(2):.1f} combined)")
check("no NaN at rest", not nan, f"clamp forces {fa:.2f} / {fb:.2f} N")
effort(joint_name="arm::drive", effort=4.0, start_time=rospy.Time(0), duration=rospy.Duration(3.0))
lo, hi, bend, force, nan, _ = watch(3.0)
check("follows a swing", not nan and hi - lo < 0.02, f"strain {lo:+.4f}..{hi:+.4f}, bend {bend:.1f} deg, force {force:.1f} N")
stops_before = state["stops"]
effort(joint_name="arm::wrist_drive", effort=-20.0, start_time=rospy.Time(0), duration=rospy.Duration(3.0))
lo, hi, bend, force, nan, _ = watch(3.0)
check("a pull calls the stop service", state["stops"] > stops_before,
      f"{state['stops'] - stops_before} calls, strain up to {hi:+.4f}, force up to {force:.1f} N")
reset()
lo, hi, bend, force, nan, (_, _, strain, pts) = watch(2.0)
check("reset re-hangs it", not nan and abs(strain) < 0.01, f"strain {strain:+.4f}, lowest {min(p[2] for p in pts):.3f}")
print("RESULT", "PASS" if not failures else f"FAIL {failures}", flush=True)
sys.exit(1 if failures else 0)
