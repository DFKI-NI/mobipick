#!/usr/bin/env python3
""": spawn the cable carrier of the wrist span as a jointed chain in Gazebo (plugin gazebo_jointed_chain) and log
its events. Sim only, off by default (launch arg cable_carrier_sim of the tables demo).

At start it waits for the robot's joint states, settles the cable model's carrier link chain at that pose (its initial
shape, so the Gazebo chain starts on the right side of every shape), builds the plugin SDF (carrier_sim.py) and spawns
it as a static holder model through /gazebo/spawn_sdf_model. Then it watches <ns>/cable_carrier/state: a strain over
~event_strain or a clamp force over ~event_force for ~event_ticks ticks is an event (logged, and one line per event in
~event_file when set). The plugin itself presses the simulated protective stop only with ~trigger_protective_stop
(default false: the Gazebo carrier replaces nothing until it catches the real stops, oscar ).

Chain values (~bending_stiffness, ~damping, ~inertia_scale, ~mass_per_metre, ~links, ~max_link_length, ~friction)
default to carrier_sim.CHAIN_DEFAULTS, PLACEHOLDERs until someone measures the real carrier; ~bend_radius 0 = the cable
model's (from its link_chain disc sizes, 3.5 cm), so both describe the same carrier.
"""
import json
import os
import time

import rospy
import yaml
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from mobipick_cable_entanglement import carrier_sim
from mobipick_cable_entanglement.cable_model import CableModel, config_from_dict


class CableCarrierSim:
    def __init__(self):
        self.ns = "/" + (rospy.get_namespace().strip("/") or "mobipick")
        urdf = rospy.get_param(rospy.get_param("~robot_description", self.ns + "/robot_description"))
        with open(rospy.get_param("~config_file")) as f:
            self.config = yaml.safe_load(f)
        prefix = rospy.get_param("~tf_prefix", self.ns.strip("/"))
        self.prefix = prefix.strip("/") + "/" if prefix.strip("/") else ""
        self.model = CableModel(urdf, config_from_dict(self.config, tf_prefix=prefix))
        self.chain = {k: rospy.get_param("~" + k, v) for k, v in carrier_sim.CHAIN_DEFAULTS.items()}
        self.chain["bend_radius"] = float(rospy.get_param("~bend_radius", 0.0))   # 0 = the model's (the disc sizes)
        self.event_strain = float(rospy.get_param("~event_strain", 0.02))   # the cable model's stop_stretch
        self.event_force = float(rospy.get_param("~event_force", 30.0))
        self.event_ticks = int(rospy.get_param("~event_ticks", 3))
        self.event_file = os.path.expanduser(rospy.get_param("~event_file", ""))
        self.chain_name = rospy.get_param("~chain_name", "cable_carrier")
        self._over = 0
        self._in_event = False
        self._peak = None

    def spawn(self):
        msg = rospy.wait_for_message(rospy.get_param("~joint_states_topic", self.ns + "/joint_states"), JointState)
        q = dict(zip(msg.name, msg.position))
        t0 = time.time()
        points = carrier_sim.initial_points_from_model(self.model, q, self.config, self.prefix)
        trigger = bool(rospy.get_param("~trigger_protective_stop", False))
        sdf = carrier_sim.plugin_sdf(
            self.model.kin, self.config, self.prefix, rospy.get_param("~host_model", self.ns.strip("/")),
            self.ns.strip("/"), chain=self.chain, initial_points=points,
            stop_service=(rospy.get_param("~protective_stop_service", self.ns + "/rqt_sim_gui/set_protective_stop")
                          if trigger else ""),
            stop_strain=self.event_strain, stop_force=self.event_force, stop_ticks=self.event_ticks,
            holder_name=self.chain_name + "_holder", chain_name=self.chain_name)
        rospy.loginfo("cable carrier sim: initial shape settled in %.1f s (%d points), EI %.3g N m^2, damping %.3g "
                      ", protective stop %s", time.time() - t0, len(points),
                      self.chain["bending_stiffness"], self.chain["damping"], "ON" if trigger else "off (events only)")
        service = rospy.get_param("~spawn_service", "/gazebo/spawn_sdf_model")
        rospy.wait_for_service(service)
        result = rospy.ServiceProxy(service, SpawnModel)(model_name=self.chain_name + "_holder", model_xml=sdf,
                                                         robot_namespace=self.ns, initial_pose=Pose(),
                                                         reference_frame="world")
        if not result.success:
            raise RuntimeError("cable carrier sim: spawn failed: " + result.status_message)
        rospy.Subscriber(self.ns + "/" + self.chain_name + "/state", Float64MultiArray, self._on_state, queue_size=1)

    def _on_state(self, msg):
        if len(msg.data) < 3:
            return
        fa, fb, strain = msg.data[:3]
        over = strain > self.event_strain or max(fa, fb) > self.event_force
        self._over = self._over + 1 if over else 0
        if self._in_event:
            self._peak = {"strain": max(self._peak["strain"], strain), "force": max(self._peak["force"], fa, fb)}
            if not over:
                self._in_event = False
                rospy.loginfo("cable carrier sim: event over, peak strain %+.4f, force %.1f N", self._peak["strain"],
                              self._peak["force"])
                self._write({"event": "end", **self._peak})
        elif self._over >= self.event_ticks:
            self._in_event = True
            self._peak = {"strain": strain, "force": max(fa, fb)}
            rospy.logwarn("cable carrier sim: EVENT strain %+.4f (limit %.3f), clamp forces %.1f / %.1f N (limit %.0f)",
                          strain, self.event_strain, fa, fb, self.event_force)
            self._write({"event": "start", "strain": strain, "force_a": fa, "force_b": fb})

    def _write(self, record):
        if not self.event_file:
            return
        record = {"stamp": rospy.get_time(), **record}
        os.makedirs(os.path.dirname(self.event_file) or ".", exist_ok=True)
        with open(self.event_file, "a") as f:
            f.write(json.dumps(record) + "\n")


if __name__ == "__main__":
    rospy.init_node("cable_carrier_sim")
    node = CableCarrierSim()
    node.spawn()
    rospy.spin()
