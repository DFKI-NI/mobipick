#!/usr/bin/env python3
# Standalone window for the simulated UR dashboard (e-stop); the same widget also runs as an rqt plugin.
import sys

import rospy
from python_qt_binding import QtWidgets

from mobipick_gazebo.simulated_ur_dashboard import SimulatedUrDashboard


def main() -> None:
    rospy.init_node("simulated_ur_dashboard")
    app = QtWidgets.QApplication(sys.argv)
    widget = SimulatedUrDashboard()
    widget.show()
    rospy.on_shutdown(app.quit)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
