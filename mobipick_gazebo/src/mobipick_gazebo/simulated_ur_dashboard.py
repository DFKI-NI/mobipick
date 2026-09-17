# Simulated UR dashboard for Gazebo: serves get_safety_mode/get_robot_mode and lets you toggle e-stop.
# While the e-stop is pressed the base and arm ros_control controllers are stopped (the diff drive brakes and
# the arm controller rejects goals) and "hold" controllers are started in their place. The hold controllers
# emulate the UR brakes: the velocity JTC leaves its last command latched in Gazebo when stopped, so the arm
# would coast; a JointGroupVelocityController starts with zero commands and the joint motors hold the pose.
# Usable both as an rqt plugin (SimulatedUrDashboardPlugin) and standalone (nodes/simulated_ur_dashboard.py).
import threading
from typing import List

import rospy
from controller_manager_msgs.srv import ListControllers, SwitchController, SwitchControllerRequest
from python_qt_binding import QtCore, QtGui, QtWidgets
from rqt_gui_py.plugin import Plugin
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
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

        self._build_ui()
        self._state_changed.connect(self._update_button)
        self._status_changed.connect(self._status.setText)
        self._update_button()

    def shutdown(self) -> None:
        for srv in (self._safety_srv, self._robot_mode_srv, self._set_estop_srv):
            srv.shutdown()

    def _handle_get_safety_mode(self, _req: GetSafetyModeRequest) -> GetSafetyModeResponse:
        mode = SafetyMode()
        mode.mode = SafetyMode.ROBOT_EMERGENCY_STOP if self._estop_pressed else SafetyMode.NORMAL
        return GetSafetyModeResponse(safety_mode=mode)

    def _handle_get_robot_mode(self, _req: GetRobotModeRequest) -> GetRobotModeResponse:
        mode = RobotMode()
        mode.mode = RobotMode.POWER_OFF if self._estop_pressed else RobotMode.RUNNING
        return GetRobotModeResponse(robot_mode=mode)

    def _handle_set_estop(self, req: SetBoolRequest) -> SetBoolResponse:
        self._set_estop(bool(req.data), "service")
        state = "PRESSED" if self._estop_pressed else "RELEASED"
        return SetBoolResponse(success=True, message=f"e-stop {state}")

    def _set_estop(self, pressed: bool, source: str) -> None:
        self._estop_pressed = pressed
        self._state_changed.emit()
        rospy.loginfo("Simulated e-stop %s via %s", "PRESSED" if pressed else "RELEASED", source)
        # controller switching talks to the controller manager; keep it off the Qt and service threads
        threading.Thread(target=self._apply_controller_lock, args=(pressed,), daemon=True).start()

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
            except (rospy.ServiceException, rospy.ROSException) as exc:
                rospy.logwarn("Simulated e-stop: controller switch failed: %s", exc)
                self._status_changed.emit(f"Controller switch failed: {exc}")

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

        layout.addWidget(self._label)
        layout.addWidget(self._button)
        layout.addWidget(self._status)

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
