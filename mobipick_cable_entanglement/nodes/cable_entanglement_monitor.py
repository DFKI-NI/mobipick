#!/usr/bin/env python3
"""Observe-only arm cable monitor: joint angles in, "is the cable entangled?" out (see doc/cable_model.md).

Simulation: when the model predicts that the real cable would make the UR stop, the node presses the sim
protective stop (mobipick_gazebo simulated_ur_dashboard), so the planner meets the same situation as on
the real robot. After the stop is released the cable counts as untangled (a person did it) and the trigger re-arms
once the arm left the over-stretched pose. Nothing is logged in the sim: its stops come from this node or the button.

Real robot: never commands anything. It logs every tick permanently (daily JSONL) and writes an event file with
the last minute of joint states, efforts and model output whenever the UR reports a protective stop, the model
predicts an entanglement, an arm trajectory aborts, or someone calls ~mark_event. These logs are the data we still
lack to calibrate the model.
"""
import collections
import datetime
import gzip
import json
import math
import os
import threading
import time

import rospy
import yaml
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String
from std_srvs.srv import SetBool, SetBoolRequest, Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion

from mobipick_cable_entanglement.cable_model import ARM_JOINTS, CableModel, config_from_dict
from mobipick_cable_entanglement.chain_reset import ChainResetPolicy
from mobipick_cable_entanglement.path_check import start_chain

try:
    from ur_dashboard_msgs.msg import SafetyMode
    from ur_dashboard_msgs.srv import GetSafetyMode
except ImportError:  # the node still works without the UR messages (no safety mode then)
    SafetyMode = None
    GetSafetyMode = None

try:
    from control_msgs.msg import FollowJointTrajectoryActionResult
except ImportError:
    FollowJointTrajectoryActionResult = None

try:
    from moveit_msgs.msg import MoveGroupActionGoal
except ImportError:  # no MoveIt messages: only the announced trajectory starts count
    MoveGroupActionGoal = None

# only needed by the sim untangle service (imported lazily there): actionlib, control_msgs FollowJointTrajectoryAction,
# controller_manager_msgs, moveit_msgs GetStateValidity, trajectory_msgs

SAFETY_NAMES = {0: "UNDEFINED", 1: "NORMAL", 2: "REDUCED", 3: "PROTECTIVE_STOP", 4: "RECOVERY", 5: "SAFEGUARD_STOP",
                6: "SYSTEM_EMERGENCY_STOP", 7: "ROBOT_EMERGENCY_STOP", 8: "VIOLATION", 9: "FAULT",
                10: "VALIDATE_JOINT_ID", 11: "UNDEFINED_SAFETY_MODE", 12: "AUTOMATIC_MODE_SAFEGUARD_STOP",
                13: "SYSTEM_THREE_POSITION_ENABLING_STOP"}
STOPPED_SAFETY = (3, 5, 12)
LEVEL_COLORS = {"ok": (0.1, 0.8, 0.2), "warning": (1.0, 0.7, 0.0), "entangled": (1.0, 0.1, 0.1)}
CARRIER_COLOR = (0.75, 0.0, 1.0)  # vivid purple (Oscar), opaque: stays visible against the planning scene
# the carrier's discs keep CARRIER_COLOR while the span is ok and take the level colour when it warns or entangles,
# so the chain itself shows what the monitor decided (also where the markers are drawn in Gazebo)


def _short(name: str) -> str:
    return name.split("/")[-1].replace("ur5_", "").replace("_joint", "")


class RealRobotLogger:
    """Permanent logs of the real robot: one JSONL.gz line per model tick, one JSON.gz file per event."""

    def __init__(self, log_dir: str, before_s: float, after_s: float, meta: dict):
        self.dir = log_dir
        self.events_dir = os.path.join(log_dir, "events")
        os.makedirs(self.events_dir, exist_ok=True)
        self.before_s = before_s
        self.after_s = after_s
        self.meta = meta
        self.samples = collections.deque()  # (t, q, v, effort) full rate, capped at 100 Hz
        self.ticks = collections.deque()  # (t, compact model state)
        self._day = None
        self._file = None
        self._last_flush = 0.0
        self._lock = threading.Lock()
        self._pending = []  # events waiting for their "after" window
        self._closed = False

    @staticmethod
    def _utc(t: float) -> datetime.datetime:
        return datetime.datetime.fromtimestamp(t, datetime.timezone.utc)

    def _open(self, t: float) -> bool:
        if self._closed:
            return False
        day = self._utc(t).strftime("%Y-%m-%d")
        if day != self._day:
            if self._file:
                self._file.close()
            self._day = day
            self._file = gzip.open(os.path.join(self.dir, f"{day}Z_cable_monitor.jsonl.gz"), "at")
            self._file.write(json.dumps({"t": t, "kind": "session_start", "meta": self.meta}) + "\n")
        return True

    def add_sample(self, t, q, v, effort):
        with self._lock:
            if self.samples and t - self.samples[-1][0] < 0.01:
                return
            self.samples.append((t, q, v, effort))
            while self.samples and t - self.samples[0][0] > self.before_s + self.after_s + 5.0:
                self.samples.popleft()

    def add_tick(self, t, row: dict):
        with self._lock:
            self.ticks.append((t, row))
            while self.ticks and t - self.ticks[0][0] > self.before_s + self.after_s + 5.0:
                self.ticks.popleft()
            if not self._open(t):
                return
            self._file.write(json.dumps(row, separators=(",", ":")) + "\n")
            if t - self._last_flush > 2.0:
                self._file.flush()
                self._last_flush = t

    def line(self, t, kind, **data):
        with self._lock:
            if not self._open(t):
                return
            self._file.write(json.dumps(dict(t=t, kind=kind, **data), separators=(",", ":")) + "\n")
            self._file.flush()

    def event(self, t, kind, detail: dict, snapshot: dict):
        """Schedule an event dump; it is written after_s later so it also holds what came next."""
        self.line(t, "event", event=kind, detail=detail)
        with self._lock:
            self._pending.append((t, kind, detail, snapshot))

    def flush_due(self, now: float, force: bool = False):
        with self._lock:
            due = [p for p in self._pending if force or now - p[0] >= self.after_s]
            self._pending = [p for p in self._pending if p not in due]
            samples, ticks = list(self.samples), list(self.ticks)
        for t, kind, detail, snapshot in due:
            name = self._utc(t).strftime("%Y-%m-%d_%H-%M-%SZ") + f"_{kind}.json.gz"
            lo, hi = t - self.before_s, t + self.after_s
            doc = {
                "event": kind, "t": t, "time_utc": self._utc(t).isoformat(), "detail": detail,
                "meta": self.meta, "window_s": [self.before_s, self.after_s], "model_at_event": snapshot,
                "joint_order": ARM_JOINTS,
                "joint_samples": [{"t": round(s[0], 3), "q": s[1], "v": s[2], "effort": s[3]}
                                  for s in samples if lo <= s[0] <= hi],
                "model_ticks": [r for s, r in ticks if lo <= s <= hi],
            }
            path = os.path.join(self.events_dir, name)
            with gzip.open(path, "wt") as f:
                json.dump(doc, f, separators=(",", ":"))
            rospy.logwarn("cable monitor: wrote event %s (%d joint samples)", path, len(doc["joint_samples"]))

    def close(self):
        self.flush_due(time.time(), force=True)
        with self._lock:
            self._closed = True
            if self._file:
                self._file.close()
                self._file = None


def _z_onto(z):
    """quaternion turning +z onto the unit vector z"""
    q = Quaternion()
    w = 1.0 + z[2]
    if w < 1e-9:
        q.x, q.w = 1.0, 0.0
    else:
        n = math.sqrt(w * w + z[1] * z[1] + z[0] * z[0])
        q.x, q.y, q.z, q.w = -z[1] / n, z[0] / n, 0.0, w / n
    return q


def _rotation_quaternion(rot):
    """Quaternion of a rotation matrix given row-major (9 numbers; its columns are the box axes)"""
    m = [[rot[0], rot[1], rot[2]], [rot[3], rot[4], rot[5]], [rot[6], rot[7], rot[8]]]
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0.0:
        s = 2.0 * math.sqrt(tr + 1.0)
        q = ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s)
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2])
        q = (0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s)
    elif m[1][1] > m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2])
        q = ((m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s)
    else:
        s = 2.0 * math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1])
        q = ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s)
    return Quaternion(*q)


NS = "/mobipick"   # set from the node namespace in CableEntanglementMonitor.__init__


class CableEntanglementMonitor:
    def __init__(self):
        global NS
        # defaults follow the node namespace (/mobipick for robot 1, as before; /mobipick2 for a second robot)
        NS = "/" + (rospy.get_namespace().strip("/") or "mobipick")
        urdf = rospy.get_param(rospy.get_param("~robot_description", NS + "/robot_description"))
        model_cfg = rospy.get_param("~model", None)
        if model_cfg is None:
            with open(rospy.get_param("~config_file")) as f:
                model_cfg = yaml.safe_load(f)
        # the links of the robot this monitor runs for (its namespace: mobipick, mobipick2, ...)
        prefix = rospy.get_param("~tf_prefix", rospy.get_namespace().strip("/") or None)
        self.cfg = config_from_dict(model_cfg, tf_prefix=prefix)
        rospy.loginfo("cable model: the wrist span decides from the %s, free length %.3f m", self.cfg.wrist_anchor,
                      next((s.free_length for s in self.cfg.spans if s.name == "wrist"), float("nan")))
        disc = self.cfg.cable_carrier.link_chain
        if self.cfg.cable_carrier.enabled:
            rospy.loginfo("cable model: carrier link chain: discs %.0f mm wide, %.1f mm high, %.1f mm apart, "
                          "joint stop "
                          "%.1f deg (bend radius %.0f mm), %s", 1000 * disc.disc_diameter,
                          1000 * disc.disc_height, 1000 * disc.disc_gap, math.degrees(disc.max_turn),
                          1000 * disc.bend_radius, "plus the geometric check" if self.cfg.decision.geometric_check else
                          "DECIDING ALONE (geometric_check false)")
        else:
            rospy.loginfo("cable model: geometric check only (cable_carrier.enabled false: no carrier link chain)")
        self.model = CableModel(urdf, self.cfg)   # raises when the carrier link chain's solver is missing (no fallback)
        self.rate = float(rospy.get_param("~rate", 10.0))
        self.mode = self._detect_mode(rospy.get_param("~mode", "auto"))
        self.sim_trigger = bool(rospy.get_param("~sim_trigger_protective_stop", True))
        self.frame = rospy.get_param("~frame_id", self.cfg.tf_prefix + self.cfg.base_link)

        self._lock = threading.Lock()
        self._q = None
        self._q_stamp = None
        # the joint states of the last seconds: a tick that comes late (a chain reset's settle took 1-2.4 s in the sim
        # check of 2026-10-01, a busy machine) catches up through them instead of feeding the chain one jump
        self._q_hist = collections.deque(maxlen=int(rospy.get_param("~joint_history", 3000)))
        self._catch_up_after = float(rospy.get_param("~catch_up_after_s", 0.3))
        self._last_stamp = None
        self._last_update = None
        self._last_level = None
        self._safety = None  # latest UR safety mode (int)
        self._armed = True
        self._stopped_by_us = False
        # sim: stretch when a person released a stop while the arm was still over-stretched; the stop is pressed
        # again as soon as the stretch grows beyond it (like the real UR, which stops again on a further pull)
        self._release_stretch = None
        self._retrigger_margin = float(rospy.get_param("~retrigger_margin", 0.005))
        self._last_state = None
        # / (oscar 2026-10-01): when the chain starts afresh, two careful flags, off in code (chain_reset.py).
        # Private parameter, else cable_model/<name> in the robot namespace (set before the node starts)
        self.resets = ChainResetPolicy(
            release_after_rest_s=float(self._cable_param("release_after_rest_s", 0.0)),
            reset_at_trajectory_start=bool(self._cable_param("reset_at_trajectory_start", False)),
            rest_tolerance_deg=float(self._cable_param("rest_tolerance_deg", 0.5)),
            release_during_rest=bool(self._cable_param("release_during_rest", False)))
        self._trajectory_source = ""
        rospy.loginfo("cable monitor: chain release after %s, reset at trajectory start %s",
                      (f"{self.resets.release_after_rest_s:g} s of rest"
                       + (" (during the rest)" if self.resets.release_during_rest else " (at the next motion)"))
                      if self.resets.release_after_rest_s > 0 else "rest off",
                      "on" if self.resets.reset_at_trajectory_start else "off")

        self.pub_state = rospy.Publisher("~state", String, queue_size=1)
        self.pub_level = rospy.Publisher("~level", String, queue_size=1, latch=True)
        self.pub_stretch = rospy.Publisher("~max_stretch", Float32, queue_size=1)
        self.pub_markers = rospy.Publisher("~markers", MarkerArray, queue_size=1)
        # the carrier's discs alone (namespace cable_discs), for a drawing of the tube outside RViz (the tables demo
        # sim draws them in Gazebo): built only while someone listens, like ~markers, and without the labels and
        # capsules that only RViz shows
        self.pub_disc_markers = rospy.Publisher("~disc_markers", MarkerArray, queue_size=1)
        self._markers_cleared = False
        # the model's collision shapes of arm and tool (capsules of config/cable_model.yaml) and the clamps, to
        # compare them in RViz with the robot meshes
        self.show_capsules = bool(rospy.get_param("~show_capsules", True))
        rospy.Service("~reset", Trigger, self._srv_reset)
        rospy.Service("~mark_event", Trigger, self._srv_mark)
        rospy.Service("~untangle_plan", Trigger, self._srv_untangle_plan)
        rospy.Service("~untangle", Trigger, self._srv_untangle)
        self._joint_names = {}  # short arm joint name -> name used in joint_states
        self._untangle_lock = threading.Lock()

        self.logger = None
        if self.mode == "real":
            log_dir = os.path.expanduser(rospy.get_param("~log_dir", "~/.ros/cable_entanglement_logs"))
            meta = {"free_length": {s.name: s.free_length for s in self.cfg.spans}, "mode": self.mode,
                    "wrist_anchor": self.cfg.wrist_anchor,
                    "chain_model": "disc_link" if self.cfg.cable_carrier.enabled else "none",
                    "decision": {"warn_stretch": self.cfg.decision.warn_stretch,
                                 "stop_stretch": self.cfg.decision.stop_stretch,
                                 "stop_torque": self.cfg.decision.stop_torque},
                    "host": os.uname().nodename}
            self.logger = RealRobotLogger(log_dir, float(rospy.get_param("~event_before_s", 60.0)),
                                          float(rospy.get_param("~event_after_s", 10.0)), meta)
            rospy.loginfo("cable monitor: real robot, logging to %s", log_dir)
        else:
            rospy.loginfo("cable monitor: simulation, protective stop trigger %s, no logging",
                          "on" if self.sim_trigger else "off")

        self._set_stop = None
        self._get_safety = None
        if self.mode == "sim":
            self._set_stop = rospy.ServiceProxy(
                rospy.get_param("~sim_protective_stop_service", NS + "/rqt_sim_gui/set_protective_stop"), SetBool)
        if GetSafetyMode is not None:
            self._get_safety = rospy.ServiceProxy(
                rospy.get_param("~safety_mode_service", NS + "/ur_hardware_interface/dashboard/get_safety_mode"),
                GetSafetyMode)
        if SafetyMode is not None:
            rospy.Subscriber(rospy.get_param("~safety_mode_topic", NS + "/ur_hardware_interface/safety_mode"),
                             SafetyMode, self._on_safety, queue_size=10)
        if FollowJointTrajectoryActionResult is not None:
            rospy.Subscriber(rospy.get_param("~trajectory_result_topic",
                                             NS + "/arm_controller/follow_joint_trajectory/result"),
                             FollowJointTrajectoryActionResult, self._on_trajectory_result, queue_size=10)
        rospy.Subscriber(rospy.get_param("~joint_states_topic", NS + "/joint_states"), JointState,
                         self._on_joints, queue_size=50)
        if self.resets.reset_at_trajectory_start:
            # an executed trajectory starts: announced by its sender (grasplan: one per executed MTC solution and per
            # guarded named move), and every executing move_group goal (MoveGroupCommander.go, e.g. robot_api)
            rospy.Subscriber(rospy.get_param("~trajectory_start_topic", NS + "/cable_model/trajectory_start"), String,
                             lambda msg: self._on_trajectory_start(msg.data or "announced"), queue_size=10)
            if MoveGroupActionGoal is not None:
                rospy.Subscriber(rospy.get_param("~move_group_goal_topic", NS + "/move_group/goal"),
                                 MoveGroupActionGoal, self._on_move_group_goal, queue_size=10)
        self._safety_poll_t = 0.0
        self._marker_t = 0.0
        rospy.Timer(rospy.Duration(1.0 / self.rate), self._tick)
        rospy.on_shutdown(self._shutdown)

    # ------------------------------------------------------------------ setup
    @staticmethod
    def _cable_param(name, default):
        return rospy.get_param("~" + name, rospy.get_param(NS + "/cable_model/" + name, default))

    @staticmethod
    def _detect_mode(mode: str) -> str:
        if mode in ("sim", "real"):
            return mode
        return "sim" if rospy.get_param("/use_sim_time", False) else "real"

    # ------------------------------------------------------------------ inputs
    def _on_joints(self, msg: JointState):
        idx = {_short(n): i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in ARM_JOINTS):
            return
        q = [msg.position[idx[j]] for j in ARM_JOINTS]
        if not self._joint_names:
            self._joint_names = {j: msg.name[idx[j]] for j in ARM_JOINTS}
        with self._lock:
            self._q = dict(zip(ARM_JOINTS, q))
            self._q_stamp = msg.header.stamp.to_sec() if msg.header.stamp else rospy.get_time()
            self._q_hist.append((self._q_stamp, self._q))
        if self.logger:
            v = [round(msg.velocity[idx[j]], 4) for j in ARM_JOINTS] if len(msg.velocity) == len(msg.name) else []
            e = [round(msg.effort[idx[j]], 4) for j in ARM_JOINTS] if len(msg.effort) == len(msg.name) else []
            self.logger.add_sample(time.time(), [round(x, 5) for x in q], v, e)

    def _on_safety(self, msg):
        self._set_safety(int(msg.mode), "topic")

    def _set_safety(self, mode: int, source: str):
        prev, self._safety = self._safety, mode
        if prev == mode:
            return
        rospy.loginfo("cable monitor: UR safety mode %s -> %s (%s)", SAFETY_NAMES.get(prev, prev),
                      SAFETY_NAMES.get(mode, mode), source)
        now = time.time()
        if self.logger:
            self.logger.line(now, "safety_mode", mode=mode, name=SAFETY_NAMES.get(mode, str(mode)))
        if mode in STOPPED_SAFETY and self.logger:
            self.logger.event(now, "protective_stop", {
                "safety_mode": SAFETY_NAMES.get(mode, mode),
                "model_predicted": self._last_state.predicted_stop if self._last_state else None,
                "model_level": self._last_state.level if self._last_state else None,
                "model_reason": self._last_state.reason if self._last_state else None}, self._snapshot())
        if prev in STOPPED_SAFETY and mode == 1:
            # a person released the stop, and in our procedure untangled the cable first
            with self._lock:
                q = dict(self._q) if self._q else None
            self._release_stretch = None
            if q:
                self.model.reset(q)
                self.resets.forget_rest(rospy.get_time(), q, ARM_JOINTS)
                stretch = self.model.evaluate(q).max_stretch
                if stretch >= self.cfg.decision.stop_stretch:
                    self._release_stretch = stretch
            self._stopped_by_us = False
            rospy.loginfo("cable monitor: stop released, cable model reset (assumed untangled)%s",
                          "" if self._release_stretch is None else
                          f"; still over-stretched ({100 * self._release_stretch:+.1f}%): any further pull stops again")

    def _on_trajectory_start(self, source: str):
        self._trajectory_source = source
        self.resets.trajectory_started()

    def _on_move_group_goal(self, msg):
        if not msg.goal.planning_options.plan_only:
            self._on_trajectory_start("move_group goal")

    def _apply_reset(self, reset):
        """the chain starts afresh (chain_reset), logged with pose and time"""
        start_chain(self.model, reset.q)
        why = ((f"after {reset.rest_s:.1f} s of rest"
                + (" (during the rest)" if self.resets.release_during_rest else "")) if reset.why == "rest" else
               f"executed trajectory starts ({self._trajectory_source})")
        q_deg = {j: round(math.degrees(v), 1) for j, v in reset.q.items()}
        rospy.loginfo("cable monitor: chain starts afresh, %s, at %s", why, q_deg)
        if self.logger:
            self.logger.line(time.time(), "chain_reset", why=reset.why, rest_s=round(reset.rest_s, 1),
                             source=self._trajectory_source if reset.why == "trajectory" else "",
                             q_deg=q_deg)

    def _on_trajectory_result(self, msg):
        status = msg.status.status
        err = msg.result.error_code
        if (status in (4, 5) or err != 0) and self.logger:  # ABORTED / REJECTED or controller error
            self.logger.event(time.time(), "trajectory_aborted", {
                "status": status, "error_code": err, "error_string": msg.result.error_string,
                "status_text": msg.status.text,
                "model_level": self._last_state.level if self._last_state else None}, self._snapshot())

    # ------------------------------------------------------------------ services
    def _srv_reset(self, _req):
        with self._lock:
            q = dict(self._q) if self._q else None
        if not q:
            return TriggerResponse(success=False, message="no joint states yet")
        self.model.reset(q)
        self.resets.forget_rest(rospy.get_time(), q, ARM_JOINTS)
        self._armed = True
        return TriggerResponse(success=True, message="cable model reset: cable assumed untangled")

    def _srv_mark(self, _req):
        if not self.logger:
            return TriggerResponse(success=False, message="event logging runs on the real robot only")
        self.logger.event(time.time(), "marked_by_operator", {}, self._snapshot())
        return TriggerResponse(success=True, message="event marked, dump follows in %.0f s" % self.logger.after_s)

    # ------------------------------------------------------------------ untangling (sim dashboard button)
    def _plan_text(self, q, steps, state, ok):
        moves = ", ".join(f"{j} {math.degrees(q[j]):.1f} -> {math.degrees(v):.1f} deg" for j, v in steps) or "none"
        return (f"moves: {moves}; cable stretch {100 * self.model.evaluate(q).max_stretch:+.1f}% -> "
                f"{100 * state.max_stretch:+.1f}%" + ("" if ok else " (still not slack enough)"))

    def _srv_untangle_plan(self, _req):
        with self._lock:
            q = dict(self._q) if self._q else None
        if not q:
            return TriggerResponse(success=False, message="no joint states yet")
        steps, state, ok = self.model.untangle_plan(q)
        return TriggerResponse(success=ok, message=self._plan_text(q, steps, state, ok))

    def _srv_untangle(self, _req):
        """Sim only, pressed by a person while the protective stop is active: move the wrist joints one at a time
        until the cable has slack again, then lock the arm again. The protective stop stays active; the person
        watches and releases it."""
        if self.mode != "sim":
            return TriggerResponse(success=False, message="untangle moves the arm: simulation only")
        if self._safety not in STOPPED_SAFETY:
            return TriggerResponse(success=False, message="press it only while the protective stop is active")
        if not self._untangle_lock.acquire(blocking=False):
            return TriggerResponse(success=False, message="untangle already running")
        try:
            return self._untangle()
        finally:
            self._untangle_lock.release()

    def _untangle(self):
        import actionlib
        from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
        from controller_manager_msgs.srv import ListControllers, SwitchController, SwitchControllerRequest
        from trajectory_msgs.msg import JointTrajectoryPoint

        with self._lock:
            q = dict(self._q) if self._q else None
        if not q or not self._joint_names:
            return TriggerResponse(success=False, message="no joint states yet")
        steps, state, ok = self.model.untangle_plan(q)
        text = self._plan_text(q, steps, state, ok)
        if not steps:
            return TriggerResponse(success=True, message="nothing to do, " + text)

        # the joint-by-joint path, sampled for a collision check (MoveIt state validity, if available)
        waypoints, cur = [], dict(q)
        for j, target in steps:
            start = cur[j]
            for i in range(1, 7):
                cur[j] = start + (target - start) * i / 6
                waypoints.append(dict(cur))
        invalid = self._first_invalid(waypoints)
        if invalid is not None:
            return TriggerResponse(success=False, message=f"not moving: collision at {invalid}; " + text)

        cm = rospy.get_param("~controller_manager", NS + "/controller_manager")
        traj_ctrl = rospy.get_param("~untangle_controller", "arm_controller")
        holds = rospy.get_param("~hold_controllers", ["arm_velocity_controller"])
        speed = float(rospy.get_param("~untangle_speed", 0.4))
        try:
            rospy.wait_for_service(f"{cm}/switch_controller", timeout=2.0)
            listing = rospy.ServiceProxy(f"{cm}/list_controllers", ListControllers)
            switch = rospy.ServiceProxy(f"{cm}/switch_controller", SwitchController)
            running = [c.name for c in listing().controller if c.state == "running"]
            stop_holds = [h for h in holds if h in running]
            if not switch(SwitchControllerRequest(start_controllers=[traj_ctrl], stop_controllers=stop_holds,
                                                  strictness=SwitchControllerRequest.BEST_EFFORT)).ok:
                return TriggerResponse(success=False, message=f"could not start {traj_ctrl}; " + text)
        except (rospy.ServiceException, rospy.ROSException) as exc:
            return TriggerResponse(success=False, message=f"controller manager: {exc}; " + text)

        result_text = ""
        try:
            action_ns = rospy.get_param("~trajectory_action_ns", "/mobipick")
            client = actionlib.SimpleActionClient(f"{action_ns}/{traj_ctrl}/follow_joint_trajectory",
                                                  FollowJointTrajectoryAction)
            if not client.wait_for_server(rospy.Duration(3.0)):
                raise RuntimeError(f"no action server for {traj_ctrl}")
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = [self._joint_names[j] for j in ARM_JOINTS]
            cur, t = dict(q), 0.5
            first = JointTrajectoryPoint(positions=[cur[j] for j in ARM_JOINTS], time_from_start=rospy.Duration(t))
            goal.trajectory.points.append(first)
            for j, target in steps:
                t += abs(target - cur[j]) / speed + 0.5
                cur[j] = target
                goal.trajectory.points.append(JointTrajectoryPoint(positions=[cur[j2] for j2 in ARM_JOINTS],
                                                                   time_from_start=rospy.Duration(t)))
            rospy.logwarn("cable monitor: UNTANGLE (sim) %s", text)
            client.send_goal(goal)
            if not client.wait_for_result(rospy.Duration(t + 10.0)):
                client.cancel_goal()
                raise RuntimeError("the untangle motion timed out")
            res = client.get_result()
            if res is not None and res.error_code != 0:
                raise RuntimeError(f"the untangle motion failed with error {res.error_code} {res.error_string}")
        except Exception as exc:  # always lock the arm again below
            result_text = str(exc)
        finally:
            try:
                running = [c.name for c in listing().controller if c.state == "running"]
                switch(SwitchControllerRequest(start_controllers=[h for h in holds if h not in running],
                                               stop_controllers=[traj_ctrl] if traj_ctrl in running else [],
                                               strictness=SwitchControllerRequest.BEST_EFFORT))
            except (rospy.ServiceException, rospy.ROSException) as exc:
                result_text += f"; could not lock the arm again: {exc}"
        if result_text:
            return TriggerResponse(success=False, message=result_text + "; " + text)
        with self._lock:
            q_now = dict(self._q)
        self.model.reset(q_now)  # the cable hangs freely again
        return TriggerResponse(success=ok, message="untangled, " + text +
                               "; the protective stop is still active: check the arm, then release it")

    def _first_invalid(self, waypoints):
        """First waypoint MoveIt reports in collision (joint angles in deg), None if all valid or no MoveIt."""
        try:
            from moveit_msgs.msg import RobotState
            from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
            name = rospy.get_param("~state_validity_service", NS + "/check_state_validity")
            rospy.wait_for_service(name, timeout=1.0)
            check = rospy.ServiceProxy(name, GetStateValidity)
        except (ImportError, rospy.ROSException):
            rospy.logwarn("cable monitor: no MoveIt state validity service, untangle without collision check")
            return None
        group = rospy.get_param("~untangle_group", "arm")
        for wp in waypoints:
            req = GetStateValidityRequest(group_name=group)
            req.robot_state = RobotState()
            req.robot_state.joint_state.name = [self._joint_names[j] for j in ARM_JOINTS]
            req.robot_state.joint_state.position = [wp[j] for j in ARM_JOINTS]
            req.robot_state.is_diff = True
            try:
                if not check(req).valid:
                    return {j: round(math.degrees(v), 1) for j, v in wp.items()}
            except rospy.ServiceException:
                return None
        return None

    # ------------------------------------------------------------------ main loop
    def _tick(self, _evt):
        with self._lock:
            q = dict(self._q) if self._q else None
            stamp = self._q_stamp
        if q is None:
            return
        now_ros = rospy.get_time()
        dt = 0.0 if self._last_update is None else max(0.0, now_ros - self._last_update)
        self._last_update = now_ros
        if self.resets.enabled:
            reset = self.resets.step(now_ros, q, ARM_JOINTS)
            if reset is not None:
                try:
                    self._apply_reset(reset)
                except Exception as exc:  # keep observing with the old chain
                    rospy.logerr(f"cable monitor: chain reset failed: {exc}")
        dt = self._catch_up(stamp, dt)
        try:
            state = self.model.update(q, dt)
        except Exception as exc:  # keep observing even if one tick fails
            rospy.logerr_throttle(10.0, f"cable monitor: model update failed: {exc}")
            return
        self._last_state = state
        d = state.to_dict()
        d["q_deg"] = {j: round(math.degrees(v), 2) for j, v in q.items()}
        d["mode"] = self.mode
        d["safety_mode"] = SAFETY_NAMES.get(self._safety, self._safety)
        self.pub_state.publish(String(data=json.dumps(d)))
        self.pub_stretch.publish(Float32(data=state.max_stretch))
        if state.level != self._last_level:
            self.pub_level.publish(String(data=state.level))
            log = rospy.logwarn if state.level != "ok" else rospy.loginfo
            log("cable monitor: %s (%s)", state.level, state.reason)
            self._on_level_change(self._last_level, state)
            self._last_level = state.level
        if not state.predicted_stop:
            self._armed = True
            self._release_stretch = None
        elif (self._release_stretch is not None and state.level == "entangled"
              and state.max_stretch > self._release_stretch + self._retrigger_margin):
            self._trigger_sim_stop(state, f"pulled further after the release ({100 * self._release_stretch:+.1f}% -> "
                                          f"{100 * state.max_stretch:+.1f}%)")
        if self.logger:
            now = time.time()
            tick = {
                "t": round(now, 3), "q": [round(q[j], 5) for j in ARM_JOINTS], "level": state.level,
                "stop": state.predicted_stop, "safety": self._safety,
                "s": {n: round(s.stretch, 4) for n, s in state.spans.items()},
                "l": {n: [round(s.l_geo, 4)] for n, s in state.spans.items()},   # L_geo (the chain: "disc")
                "hooked": sorted({h for s in state.spans.values() for h in s.hooked_on}),
                "tau": [round(state.torques[j], 3) for j in ARM_JOINTS]}
            if any(s.l_disc is not None for s in state.spans.values()):
                # per span [L_disc, L_disc_topology, twist deg, strain] and the tool parts the carrier link chain
                # is caught on
                tick["disc"] = {n: [round(s.l_disc, 4), round(s.l_disc_topology, 4), round(s.twist_deg, 1),
                                    round(s.disc_strain, 4)]
                                for n, s in state.spans.items() if s.l_disc is not None}
                tick["disc_hooked"] = sorted({h for s in state.spans.values() for h in (s.disc_hooked_on or [])})
            self.logger.add_tick(now, tick)
            self.logger.flush_due(now)
        self._poll_safety()
        if now_ros - self._marker_t > 0.05:  # the discs: every tick
            self._marker_t = now_ros
            self._publish_markers(q, state)

    def _catch_up(self, stamp, dt):
        """after a late tick: one model update per 0.1 s of the joint states received since the last update (the
        tick's own update follows with the latest pose); a single update over a large joint jump pulls the chain taut
        (+37 % after the 2.4 s reset settle at the observe pose in the sim, 2026-10-01). Returns the dt left for the
        tick's own update."""
        last, self._last_stamp = self._last_stamp, stamp
        if last is None or stamp is None or stamp - last <= self._catch_up_after:
            return dt
        with self._lock:
            hist = [(t, q) for t, q in self._q_hist if last < t < stamp]
        prev, steps = last, 0
        for t, q in hist:
            if t - prev < 0.1 - 1e-6:
                continue
            try:
                self.model.update(q, t - prev)
            except Exception as exc:  # the tick's own update still runs
                rospy.logerr_throttle(10.0, f"cable monitor: catch-up update failed: {exc}")
                return dt
            prev, steps = t, steps + 1
        if steps:
            rospy.loginfo_throttle(5.0, f"cable monitor: caught up {stamp - last:.1f} s of joint states in {steps} "
                                        "updates (late tick)")
        return max(0.0, stamp - prev)

    def _on_level_change(self, prev, state):
        if state.level != "entangled":
            return
        if self.logger:
            self.logger.event(time.time(), "predicted_entanglement", {"reason": state.reason}, self._snapshot())
        if self._armed:
            self._trigger_sim_stop(state, "")

    def _trigger_sim_stop(self, state, why: str):
        if self.mode != "sim" or not self.sim_trigger or self._stopped_by_us or self._safety in STOPPED_SAFETY:
            return
        try:
            self._set_stop.wait_for_service(timeout=1.0)
            resp = self._set_stop(SetBoolRequest(data=True))
            self._stopped_by_us = True
            self._armed = False
            self._release_stretch = None
            rospy.logwarn("cable monitor: SIM PROTECTIVE STOP triggered (%s%s): %s", state.reason,
                          f"; {why}" if why else "", resp.message)
        except (rospy.ServiceException, rospy.ROSException) as exc:
            rospy.logerr("cable monitor: could not trigger the sim protective stop: %s", exc)

    def _poll_safety(self):
        """Without a safety_mode topic (the sim), poll the dashboard service at 1 Hz."""
        if self._get_safety is None or (self.mode == "real" and self._safety is not None):
            return
        now = time.time()
        if now - self._safety_poll_t < 1.0:
            return
        self._safety_poll_t = now
        try:
            self._get_safety.wait_for_service(timeout=0.05)
            self._set_safety(int(self._get_safety().safety_mode.mode), "service")
        except (rospy.ServiceException, rospy.ROSException):
            pass

    def _snapshot(self) -> dict:
        snap = self.model.snapshot()
        if self._last_state:
            snap["state"] = self._last_state.to_dict()
        return snap

    # ------------------------------------------------------------------ markers
    def _publish_markers(self, q, state):
        full = self.pub_markers.get_num_connections() > 0
        discs_wanted = self.pub_disc_markers.get_num_connections() > 0
        if not (full or discs_wanted):
            return
        arr, disc_arr = MarkerArray(), MarkerArray()
        stamp = rospy.Time.now()
        if full and not self._markers_cleared:   # markers of an earlier monitor (e.g. the removed radius rope line) go
            arr.markers.append(Marker(action=Marker.DELETEALL))
            self._markers_cleared = True
        discs = self.model.disc_points()   # the carrier is drawn as its discs (none without the chain)
        for i, (name, pts) in enumerate(discs.items()):
            span = state.spans[name]
            lvl = "entangled" if span.stretch >= self.cfg.decision.stop_stretch else (
                "warning" if span.stretch >= self.cfg.decision.warn_stretch or span.hooked_on else "ok")
            markers = self._disc_markers(i, pts, stamp, lvl)
            disc_arr.markers.extend(m for m in markers if m.ns == "cable_discs")
            if not full:
                continue
            arr.markers.extend(markers)
            t = Marker(type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, id=100 + i, ns="cable_text")
            t.header.frame_id, t.header.stamp = self.frame, stamp
            mid = pts[len(pts) // 2]
            t.pose.position = Point(mid[0], mid[1], mid[2] + 0.06)
            t.pose.orientation.w = 1.0
            t.scale.z = 0.035
            t.color.r, t.color.g, t.color.b = LEVEL_COLORS[lvl]
            t.color.a = 1.0
            t.text = f"{name} {100 * span.stretch:+.0f}%" + (f" caught: {','.join(span.hooked_on)}" if span.hooked_on else "")
            if span.disc_strain is not None and span.disc_strain >= self.cfg.cable_carrier.link_chain.strain_tolerance:
                t.text += f" chain taut {100 * span.disc_strain:+.0f}%"
            arr.markers.append(t)
        if discs_wanted:
            self.pub_disc_markers.publish(disc_arr)
        if not full:
            return
        if self.show_capsules:
            arr.markers.extend(self._capsule_markers(q, stamp))
        self.pub_markers.publish(arr)

    def _disc_markers(self, span_index, joints, stamp, level="ok"):
        """one cylinder per link of the carrier link chain (at the middle of the link, across it) and the links as a
        line; the discs in CARRIER_COLOR while the span is ok, else in the colour of its level"""
        disc = self.cfg.cable_carrier.link_chain
        out = []
        line = Marker(type=Marker.LINE_STRIP, action=Marker.ADD, id=span_index, ns="cable_links")
        line.header.frame_id, line.header.stamp = self.frame, stamp
        line.scale.x = 0.006
        line.color.r, line.color.g, line.color.b, line.color.a = 0.1, 0.1, 0.1, 1.0
        line.pose.orientation.w = 1.0
        line.points = [Point(*p) for p in joints]
        out.append(line)
        for k in range(len(joints) - 1):
            a, b = joints[k], joints[k + 1]
            axis = [bi - ai for ai, bi in zip(a, b)]
            length = math.sqrt(sum(v * v for v in axis))
            if length < 1e-9:
                continue
            m = Marker(type=Marker.CYLINDER, action=Marker.ADD, id=1000 * span_index + k, ns="cable_discs")
            m.header.frame_id, m.header.stamp = self.frame, stamp
            m.pose.position = Point(*[(ai + bi) / 2.0 for ai, bi in zip(a, b)])
            m.pose.orientation = _z_onto([v / length for v in axis])
            m.scale.x = m.scale.y = disc.disc_diameter
            m.scale.z = disc.disc_height
            m.color.r, m.color.g, m.color.b = CARRIER_COLOR if level == "ok" else LEVEL_COLORS[level]
            m.color.a = 1.0
            out.append(m)
        return out

    def _capsule_markers(self, q, stamp):
        """semi-transparent capsules (cylinder + end spheres) and flat cylinders with the housing radius, clamps as
        black spheres"""
        _, caps = self.model.anchors(q)
        out = []

        def marker(kind, mid, ns):
            m = Marker(type=kind, action=Marker.ADD, id=mid, ns=ns)
            m.header.frame_id, m.header.stamp = self.frame, stamp
            m.pose.orientation.w = 1.0
            return m

        for i, (a, b, r, core_radius, box) in enumerate(caps):
            radius = r - self.cfg.cable_carrier_radius  # the model widens every shape by the carrier radius
            axis = [bi - ai for ai, bi in zip(a, b)]
            length = math.sqrt(sum(v * v for v in axis))
            if box[0] > 0.0:   # a rounded box (core + edge), drawn as a box of its full size
                cube = marker(Marker.CUBE, 3 * i + 2, "collision_capsules")
                cube.pose.position = Point(*a)
                cube.pose.orientation = _rotation_quaternion(box[3:])
                cube.scale.x, cube.scale.y, cube.scale.z = (2.0 * (h + radius) for h in box[:3])
                cube.color.r, cube.color.g, cube.color.b, cube.color.a = 0.2, 0.6, 1.0, 0.2
                out.append(cube)
                continue
            if core_radius > 0.0:   # a flat cylinder (core + rounded edge): drawn between its faces
                u = [v / length for v in axis] if length > 1e-9 else [0.0, 0.0, 1.0]
                cyl = marker(Marker.CYLINDER, 3 * i + 2, "collision_capsules")
                cyl.pose.position = Point(*[(ai + bi) / 2.0 for ai, bi in zip(a, b)])
                cyl.pose.orientation = _z_onto(u)
                cyl.scale.x = cyl.scale.y = 2.0 * (core_radius + radius)
                cyl.scale.z = length + 2.0 * radius
                cyl.color.r, cyl.color.g, cyl.color.b, cyl.color.a = 0.2, 0.6, 1.0, 0.2
                out.append(cyl)
                continue
            for k, end in enumerate((a, b)):
                sph = marker(Marker.SPHERE, 3 * i + k, "collision_capsules")
                sph.pose.position = Point(*end)
                sph.scale.x = sph.scale.y = sph.scale.z = 2.0 * radius
                sph.color.r, sph.color.g, sph.color.b, sph.color.a = 0.2, 0.6, 1.0, 0.2
                out.append(sph)
            if length < 1e-6:
                continue
            cyl = marker(Marker.CYLINDER, 3 * i + 2, "collision_capsules")
            cyl.pose.position = Point(*[(ai + bi) / 2.0 for ai, bi in zip(a, b)])
            cyl.pose.orientation = _z_onto([v / length for v in axis])
            cyl.scale.x = cyl.scale.y = 2.0 * radius
            cyl.scale.z = length
            cyl.color.r, cyl.color.g, cyl.color.b, cyl.color.a = 0.2, 0.6, 1.0, 0.2
            out.append(cyl)
        # the clamps where the carrier is really held: the deciding anchors of the wrist span
        # sit
        # elsewhere on purpose (the support edge and the tip of the guide link, MoveIt's mockup of the carrier) and drew
        # a clamp at the tip, which read as "the carrier is anchored there"
        for i, (name, (a, b)) in enumerate(self.model.carrier_clamps(q).items()):
            for k, end in enumerate((a, b)):
                c = marker(Marker.SPHERE, 2 * i + k, "clamps")
                c.pose.position = Point(*end)
                c.scale.x = c.scale.y = c.scale.z = 0.025
                c.color.a = 1.0
                out.append(c)
        return out

    def _shutdown(self):
        if self.logger:
            self.logger.close()


if __name__ == "__main__":
    rospy.init_node("cable_entanglement_monitor")
    CableEntanglementMonitor()
    rospy.spin()
