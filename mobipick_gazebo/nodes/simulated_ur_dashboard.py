#!/usr/bin/env python3
# Simulated UR dashboard for Gazebo: serves get_safety_mode/get_robot_mode and lets you toggle e-stop.
import sys
from typing import Tuple

import rospy
from PyQt5 import QtCore, QtGui, QtWidgets
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
    def __init__(self):
        super().__init__()
        self._estop_pressed = False
        self._ns = _normalize_ns(rospy.get_param("~robot_namespace", "mobipick"))

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
        self._update_button()

    def _handle_get_safety_mode(self, _req: GetSafetyModeRequest) -> GetSafetyModeResponse:
        mode = SafetyMode()
        mode.mode = SafetyMode.ROBOT_EMERGENCY_STOP if self._estop_pressed else SafetyMode.NORMAL
        return GetSafetyModeResponse(safety_mode=mode)

    def _handle_get_robot_mode(self, _req: GetRobotModeRequest) -> GetRobotModeResponse:
        mode = RobotMode()
        mode.mode = RobotMode.POWER_OFF if self._estop_pressed else RobotMode.RUNNING
        return GetRobotModeResponse(robot_mode=mode)

    def _handle_set_estop(self, req: SetBoolRequest) -> SetBoolResponse:
        self._estop_pressed = bool(req.data)
        self._update_button()
        state = "PRESSED" if self._estop_pressed else "RELEASED"
        rospy.loginfo("Simulated e-stop set to %s", state)
        return SetBoolResponse(success=True, message=f"e-stop {state}")

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

        layout.addWidget(self._label)
        layout.addWidget(self._button)

    def _toggle_estop(self) -> None:
        self._estop_pressed = not self._estop_pressed
        self._update_button()
        state = "PRESSED" if self._estop_pressed else "RELEASED"
        rospy.loginfo("E-stop %s via GUI", state)

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

def main() -> None:
    rospy.init_node("simulated_ur_dashboard")
    app = QtWidgets.QApplication(sys.argv)
    widget = SimulatedUrDashboard()
    widget.show()
    rospy.on_shutdown(app.quit)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
