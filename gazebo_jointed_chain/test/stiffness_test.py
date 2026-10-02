#!/usr/bin/env python3
"""gazebo_jointed_chain bending stiffness (B0 a, b of #263) against gzserver running cantilever_world.sdf (generated
from cantilever_world.sdf.in with an EI value): (a) the free end's height below the clamp after 5 s at rest (a free-hinge
chain hangs straight down, -0.30 m); (b) a push at the middle link for 2 s, then release: the free end's way back
(fraction of the push recovered after 3 s) and the time to 63 % of it (the spring's time constant). One JSON line.
  python3 stiffness_test.py --ei 0.05
"""
import argparse
import json
import time

import rospy
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Point, Vector3, Wrench
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray

ap = argparse.ArgumentParser()
ap.add_argument("--ei", type=float, required=True)
ap.add_argument("--push", type=float, default=0.5, help="force at the middle link [N], downwards")
a = ap.parse_args()
rospy.init_node("stiffness_test")
state = {"msg": None, "clock": None}
rospy.Subscriber("/free_chain/state", Float64MultiArray, lambda m: state.__setitem__("msg", m), queue_size=1)
rospy.Subscriber("/clock", Clock, lambda m: state.__setitem__("clock", m.clock.to_sec()), queue_size=1)
rospy.wait_for_service("/gazebo/apply_body_wrench", 60)
wrench = rospy.ServiceProxy("/gazebo/apply_body_wrench", ApplyBodyWrench)
t0 = time.monotonic()
while (state["msg"] is None or state["clock"] is None) and time.monotonic() - t0 < 60:
    time.sleep(0.1)


def tip():
    d = state["msg"].data
    n = int(d[3])
    return d[4 + 3 * (n - 1) + 2], any(v != v for v in d)   # z of the last point (the free end), NaN


def wait(seconds, samples=None):
    start = state["clock"]
    while state["clock"] - start < seconds:
        if samples is not None:
            samples.append((state["clock"] - start, tip()[0]))
        time.sleep(0.02)


wait(5.0)
rest, nan = tip()
wrench(body_name="free_chain::l4", reference_frame="world", reference_point=Point(0, 0, 0),
       wrench=Wrench(force=Vector3(0, 0, -abs(a.push)), torque=Vector3(0, 0, 0)), start_time=rospy.Time(0),
       duration=rospy.Duration(2.0))
wait(2.0)
pushed, _ = tip()
back = []
wait(3.0, back)
end, nan2 = tip()
moved = rest - pushed
recovered = (end - pushed) / moved if abs(moved) > 1e-4 else None
tau = next((t for t, z in back if moved and (z - pushed) / moved >= 0.632), None)
print(json.dumps(dict(ei=a.ei, rest_tip_z=round(rest, 4), pushed_tip_z=round(pushed, 4), end_tip_z=round(end, 4),
                      recovered=None if recovered is None else round(recovered, 3), tau_s=tau and round(tau, 2),
                      nan=nan or nan2)), flush=True)
