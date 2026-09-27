# Simulated UR dashboard for Gazebo: serves get_safety_mode/get_robot_mode and lets you toggle e-stop.
# While the e-stop is pressed the base and arm ros_control controllers are stopped (the diff drive brakes and
# the arm controller rejects goals) and "hold" controllers are started in their place. The hold controllers
# emulate the UR brakes: the velocity JTC leaves its last command latched in Gazebo when stopped, so the arm
# would coast; a JointGroupVelocityController starts with zero commands and the joint motors hold the pose.
# Usable both as an rqt plugin (SimulatedUrDashboardPlugin) and standalone (nodes/simulated_ur_dashboard.py).
import math
import threading
from typing import Dict, List, Optional

import actionlib
import rospy
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryActionGoal,
    FollowJointTrajectoryActionResult,
    FollowJointTrajectoryGoal,
)
from controller_manager_msgs.srv import ListControllers, SwitchController, SwitchControllerRequest
from python_qt_binding import QtCore, QtGui, QtWidgets
from rqt_gui_py.plugin import Plugin
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse, Trigger
from ur_dashboard_msgs.msg import RobotMode, SafetyMode
from ur_dashboard_msgs.srv import (
    GetRobotMode,
    GetRobotModeRequest,
    GetRobotModeResponse,
    GetSafetyMode,
    GetSafetyModeRequest,
    GetSafetyModeResponse,
)


def _normalize_ns(ns: str) -> str:
    ns_clean = ns.strip("/")
    return f"/{ns_clean}/" if ns_clean else "/"


class SimulatedUrDashboard(QtWidgets.QWidget):
    # service callbacks and controller switching run in non-Qt threads; route widget updates through signals
    _state_changed = QtCore.pyqtSignal()
    _status_changed = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._estop_pressed = False
        # protective stop (e.g. an entangled cable on the real arm): the arm is locked like with the e-stop, but
        # the robot stays powered and get_safety_mode reports PROTECTIVE_STOP
        self._protective_stop = False
        self._ns = _normalize_ns(rospy.get_param("~robot_namespace", "mobipick"))
        # controllers to stop while the e-stop is pressed (only the ones currently running are touched)
        self._estop_controllers: List[str] = rospy.get_param(
            "~estop_controllers",
            ["mobile_base_controller", "arm_controller", "arm_velocity_controller", "arm_position_controller"],
        )
        # controllers started while the e-stop is pressed and stopped again on release (brake emulation)
        self._hold_controllers: List[str] = rospy.get_param("~estop_hold_controllers", ["arm_velocity_controller"])
        cm_ns = rospy.get_param("~controller_manager", f"{self._ns}controller_manager")
        self._list_controllers = rospy.ServiceProxy(f"{cm_ns}/list_controllers", ListControllers)
        self._switch_controller = rospy.ServiceProxy(f"{cm_ns}/switch_controller", SwitchController)
        self._stopped_controllers: List[str] = []
        self._switch_lock = threading.Lock()
        # "untangle cable" button: while the protective stop is active, the cable monitor
        # (mobipick_sim_cable_entanglement, #118) moves the wrist joints one at a time until the arm cable has slack
        # again; the protective stop stays active, the person watches and releases it
        self._untangle_service = rospy.get_param(
            "~untangle_service", "/mobipick/cable_entanglement_monitor/untangle")
        self._untangle_running = False
        # like the real UR, releasing a protective stop can let the interrupted arm trajectory continue: the rest of
        # the trajectory that ran when the stop hit is resumed on release, unless the arm was moved meanwhile (e.g.
        # UNTANGLE CABLE), the e-stop was pressed, or ~resume_trajectory_after_release is false
        self._resume_after_release = bool(rospy.get_param("~resume_trajectory_after_release", True))
        self._resume_max_offset = math.radians(float(rospy.get_param("~resume_max_joint_offset_deg", 2.0)))
        self._traj_action = rospy.get_param("~arm_trajectory_action", f"{self._ns}arm_controller/follow_joint_trajectory")
        self._active_traj = None  # (goal id, trajectory, start time) of the running arm trajectory
        self._paused_traj = None  # (trajectory, elapsed s, joint positions) snapshot taken when the stop hit
        self._joints: Dict[str, float] = {}
        self._traj_client: Optional[actionlib.SimpleActionClient] = None
        rospy.Subscriber(f"{self._traj_action}/goal", FollowJointTrajectoryActionGoal, self._on_traj_goal, queue_size=5)
        rospy.Subscriber(f"{self._traj_action}/result", FollowJointTrajectoryActionResult, self._on_traj_result,
                         queue_size=5)
        rospy.Subscriber(f"{self._ns}joint_states", JointState, self._on_joints, queue_size=5)

        self._safety_srv = rospy.Service(
            f"{self._ns}ur_hardware_interface/dashboard/get_safety_mode",
            GetSafetyMode,
            self._handle_get_safety_mode,
        )
        self._robot_mode_srv = rospy.Service(
            f"{self._ns}ur_hardware_interface/dashboard/get_robot_mode",
            GetRobotMode,
            self._handle_get_robot_mode,
        )
        self._set_estop_srv = rospy.Service("~set_estop", SetBool, self._handle_set_estop)
        self._set_protective_srv = rospy.Service("~set_protective_stop", SetBool, self._handle_set_protective_stop)

        self._build_ui()
        self._state_changed.connect(self._update_button)
        self._status_changed.connect(self._status.setText)
        self._update_button()

    def shutdown(self) -> None:
        for srv in (self._safety_srv, self._robot_mode_srv, self._set_estop_srv, self._set_protective_srv):
            srv.shutdown()

    def _handle_get_safety_mode(self, _req: GetSafetyModeRequest) -> GetSafetyModeResponse:
        mode = SafetyMode()
        if self._estop_pressed:
            mode.mode = SafetyMode.ROBOT_EMERGENCY_STOP
        elif self._protective_stop:
            mode.mode = SafetyMode.PROTECTIVE_STOP
        else:
            mode.mode = SafetyMode.NORMAL
        return GetSafetyModeResponse(safety_mode=mode)

    def _handle_get_robot_mode(self, _req: GetRobotModeRequest) -> GetRobotModeResponse:
        mode = RobotMode()
        mode.mode = RobotMode.POWER_OFF if self._estop_pressed else RobotMode.RUNNING
        return GetRobotModeResponse(robot_mode=mode)

    def _handle_set_estop(self, req: SetBoolRequest) -> SetBoolResponse:
        self._set_estop(bool(req.data), "service")
        state = "PRESSED" if self._estop_pressed else "RELEASED"
        return SetBoolResponse(success=True, message=f"e-stop {state}")

    def _handle_set_protective_stop(self, req: SetBoolRequest) -> SetBoolResponse:
        self._set_protective_stop(bool(req.data), "service")
        state = "ACTIVE" if self._protective_stop else "RELEASED"
        return SetBoolResponse(success=True, message=f"protective stop {state}")

    def _set_estop(self, pressed: bool, source: str) -> None:
        was_locked = self._estop_pressed or self._protective_stop
        self._estop_pressed = pressed
        rospy.loginfo("Simulated e-stop %s via %s", "PRESSED" if pressed else "RELEASED", source)
        self._apply_lock_change(was_locked)

    def _set_protective_stop(self, active: bool, source: str) -> None:
        was_locked = self._estop_pressed or self._protective_stop
        if active and not was_locked:
            self._pause_trajectory()
        self._protective_stop = active
        rospy.loginfo("Simulated protective stop %s via %s", "ACTIVE" if active else "RELEASED", source)
        self._apply_lock_change(was_locked)

    def _apply_lock_change(self, was_locked: bool) -> None:
        self._state_changed.emit()
        locked = self._estop_pressed or self._protective_stop
        if locked != was_locked:
            # controller switching talks to the controller manager; keep it off the Qt and service threads
            threading.Thread(target=self._apply_controller_lock, args=(locked,), daemon=True).start()

    def _apply_controller_lock(self, pressed: bool) -> None:
        with self._switch_lock:
            try:
                if pressed:
                    loaded = {c.name: c.state for c in self._list_controllers().controller}
                    running = [n for n in self._estop_controllers if loaded.get(n) == "running"]
                    if running:
                        self._switch(stop=running)
                    self._stopped_controllers = running
                    # a hold controller that is not loaded (e.g. position arm mode) is skipped: those hold anyway
                    hold = [n for n in self._hold_controllers if n in loaded]
                    if hold:
                        self._switch(start=hold)
                    self._status_changed.emit(
                        "Locked: " + (", ".join(running) if running else "no matching controller running")
                        + (f"\nHolding with: {', '.join(hold)}" if hold else "")
                    )
                else:
                    loaded = {c.name: c.state for c in self._list_controllers().controller}
                    hold = [n for n in self._hold_controllers if loaded.get(n) == "running"]
                    if hold:
                        self._switch(stop=hold)
                    if self._stopped_controllers:
                        self._switch(start=self._stopped_controllers)
                    self._stopped_controllers = []
                    self._status_changed.emit("Controllers running")
                    self._resume_trajectory()
            except (rospy.ServiceException, rospy.ROSException) as exc:
                rospy.logwarn("Simulated e-stop: controller switch failed: %s", exc)
                self._status_changed.emit(f"Controller switch failed: {exc}")

    # ------------------------------------------------------------------ interrupted trajectory
    def _on_traj_goal(self, msg: FollowJointTrajectoryActionGoal) -> None:
        stamp = msg.goal.trajectory.header.stamp
        start = stamp if stamp and not stamp.is_zero() else rospy.Time.now()
        self._active_traj = (msg.goal_id.id, msg.goal.trajectory, start)

    def _on_traj_result(self, msg: FollowJointTrajectoryActionResult) -> None:
        if self._active_traj and msg.status.goal_id.id == self._active_traj[0]:
            self._active_traj = None

    def _on_joints(self, msg: JointState) -> None:
        self._joints.update(zip(msg.name, msg.position))

    def _pause_trajectory(self) -> None:
        """Snapshot of the running arm trajectory when the protective stop hits (the controller stop aborts it)."""
        self._paused_traj = None
        if not self._resume_after_release or self._active_traj is None:
            return
        _, traj, start = self._active_traj
        elapsed = (rospy.Time.now() - start).to_sec()
        if traj.points and elapsed < traj.points[-1].time_from_start.to_sec():
            self._paused_traj = (traj, elapsed, {n: self._joints.get(n) for n in traj.joint_names})

    def _resume_trajectory(self) -> None:
        paused, self._paused_traj = self._paused_traj, None
        if paused is None or self._estop_pressed or self._protective_stop:
            return
        traj, elapsed, q_stop = paused
        moved = [n for n in traj.joint_names
                 if q_stop.get(n) is None or abs(self._joints.get(n, 1e9) - q_stop[n]) > self._resume_max_offset]
        if moved:
            rospy.loginfo("Simulated dashboard: arm moved during the stop (%s), interrupted trajectory dropped",
                          ", ".join(moved))
            return
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = list(traj.joint_names)
        lead = 0.3
        for p in traj.points:
            t = p.time_from_start.to_sec() - elapsed
            if t > 0.0:
                p.time_from_start = rospy.Duration(t + lead)
                goal.trajectory.points.append(p)
        if not goal.trajectory.points:
            return
        try:
            if self._traj_client is None:
                self._traj_client = actionlib.SimpleActionClient(self._traj_action, FollowJointTrajectoryAction)
            if not self._traj_client.wait_for_server(rospy.Duration(2.0)):
                raise rospy.ROSException("no trajectory action server")
            self._traj_client.send_goal(goal)
            text = (f"Resumed the interrupted arm trajectory ({goal.trajectory.points[-1].time_from_start.to_sec():.1f} s "
                    "left), like the real UR after a release")
        except rospy.ROSException as exc:
            text = f"Could not resume the interrupted trajectory: {exc}"
        rospy.logwarn("Simulated dashboard: %s", text)
        self._status_changed.emit(text)

    def _switch(self, start: List[str] = (), stop: List[str] = ()) -> None:
        self._switch_controller.wait_for_service(timeout=2.0)
        req = SwitchControllerRequest(
            start_controllers=list(start),
            stop_controllers=list(stop),
            strictness=SwitchControllerRequest.BEST_EFFORT,
        )
        resp = self._switch_controller(req)
        if not resp.ok:
            raise rospy.ServiceException(f"switch_controller rejected start={list(start)} stop={list(stop)}")
        rospy.loginfo("Simulated e-stop: started %s, stopped %s", list(start), list(stop))

    def _build_ui(self) -> None:
        self.setWindowTitle("Simulated UR Dashboard - E-Stop")
        self.resize(240, 140)
        layout = QtWidgets.QVBoxLayout(self)

        self._label = QtWidgets.QLabel("E-Stop State:")
        font = self._label.font()
        font.setPointSize(12)
        font.setBold(True)
        self._label.setFont(font)
        self._label.setAlignment(QtCore.Qt.AlignCenter)

        self._button = QtWidgets.QPushButton()
        self._button.setCheckable(True)
        self._button.setMinimumHeight(60)
        self._button.clicked.connect(self._toggle_estop)

        self._status = QtWidgets.QLabel("Controllers running")
        self._status.setAlignment(QtCore.Qt.AlignCenter)
        self._status.setWordWrap(True)

        self._protective_button = QtWidgets.QPushButton()
        self._protective_button.setCheckable(True)
        self._protective_button.setMinimumHeight(40)
        self._protective_button.setToolTip("Simulate a UR protective stop (e.g. an entangled cable)")
        self._protective_button.clicked.connect(
            lambda: self._set_protective_stop(not self._protective_stop, "GUI"))

        self._untangle_button = QtWidgets.QPushButton("UNTANGLE CABLE")
        self._untangle_button.setMinimumHeight(40)
        self._untangle_button.setToolTip(
            "While the protective stop is active: move the wrist joints one at a time until the arm cable has "
            "slack again (cable monitor). The protective stop stays active: watch the arm, then release it.")
        self._untangle_button.clicked.connect(self._start_untangle)

        layout.addWidget(self._label)
        layout.addWidget(self._button)
        layout.addWidget(self._protective_button)
        layout.addWidget(self._untangle_button)
        layout.addWidget(self._status)

    def _start_untangle(self) -> None:
        if self._untangle_running or not self._protective_stop or self._estop_pressed:
            return
        self._untangle_running = True
        self._state_changed.emit()
        self._status_changed.emit("Untangling the cable ...")
        threading.Thread(target=self._untangle, daemon=True).start()

    def _untangle(self) -> None:
        try:
            rospy.wait_for_service(self._untangle_service, timeout=2.0)
            resp = rospy.ServiceProxy(self._untangle_service, Trigger)()
            text = ("Untangled: " if resp.success else "Untangle failed: ") + resp.message
        except (rospy.ServiceException, rospy.ROSException) as exc:
            text = f"Untangle unavailable (is the cable monitor running?): {exc}"
        rospy.loginfo("Simulated dashboard: %s", text)
        self._untangle_running = False
        self._status_changed.emit(text)
        self._state_changed.emit()

    def _toggle_estop(self) -> None:
        self._set_estop(not self._estop_pressed, "GUI")

    def _update_button(self) -> None:
        palette = self._button.palette()
        if self._estop_pressed:
            self._button.setText("E-STOP PRESSED")
            palette.setColor(QtGui.QPalette.Button, QtGui.QColor(200, 0, 0))
        else:
            self._button.setText("E-STOP RELEASED")
            palette.setColor(QtGui.QPalette.Button, QtGui.QColor(0, 160, 0))
        self._button.setPalette(palette)
        self._button.setAutoFillBackground(True)
        self._button.update()
        self._button.setChecked(self._estop_pressed)
        palette = self._protective_button.palette()
        if self._protective_stop:
            self._protective_button.setText("PROTECTIVE STOP ACTIVE")
            palette.setColor(QtGui.QPalette.Button, QtGui.QColor(230, 140, 0))
        else:
            self._protective_button.setText("PROTECTIVE STOP: none")
            palette.setColor(QtGui.QPalette.Button, QtGui.QColor(0, 160, 0))
        self._protective_button.setPalette(palette)
        self._protective_button.setAutoFillBackground(True)
        self._protective_button.setChecked(self._protective_stop)
        self._untangle_button.setEnabled(self._protective_stop and not self._estop_pressed
                                         and not self._untangle_running)
        self._untangle_button.setText("UNTANGLING ..." if self._untangle_running else "UNTANGLE CABLE")


class SimulatedUrDashboardPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("SimulatedUrDashboard")
        self._widget = SimulatedUrDashboard()
        self._widget.setObjectName("SimulatedUrDashboardUi")
        if context.serial_number() > 1:
            self._widget.setWindowTitle(f"{self._widget.windowTitle()} ({context.serial_number()})")
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown()
