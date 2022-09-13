#!/usr/bin/env python3
from control_msgs.msg import FollowJointTrajectoryActionGoal, JointTrajectoryControllerState
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import rospy
import sys
import threading


def add_goal(msg):
    trajectory_start_time = msg.goal.trajectory.header.stamp.to_sec()
    if trajectory_start_time == 0.0:
        trajectory_start_time = rospy.Time.now().to_sec()
    with goals_lock:
        goals.append((trajectory_start_time, msg.goal))


def add_controller_state(msg):
    with controller_states_lock:
        controller_states.append(msg)


def update(foo):
    # Plot any new tractories
    planned_times = []
    planned_positions = []
    with goals_lock:
        for (trajectory_start_time, goal) in goals:
            trajectory = goal.trajectory
            joint_index = trajectory.joint_names.index(g_joint_name)
            planned_times.extend(
                np.array([point.time_from_start.to_sec() for point in trajectory.points]) + trajectory_start_time
            )
            planned_positions.extend(np.array([point.positions[joint_index] for point in trajectory.points]))
    planned_positions_line.set_xdata(np.array(planned_times))
    planned_positions_line.set_ydata(np.array(planned_positions))

    # Plot controller states
    times = []
    desired_positions = []
    actual_positions = []
    errors = []
    with controller_states_lock:
        for state in controller_states:
            joint_index = state.joint_names.index(g_joint_name)
            times.append(state.header.stamp.to_sec())
            desired_positions.append(state.desired.positions[joint_index])
            actual_positions.append(state.actual.positions[joint_index])
            errors.append(state.desired.positions[joint_index] - state.actual.positions[joint_index])

    desired_positions_line.set_xdata(np.array(times))
    desired_positions_line.set_ydata(np.array(desired_positions))
    actual_positions_line.set_xdata(np.array(times))
    actual_positions_line.set_ydata(np.array(actual_positions))
    errors_line.set_xdata(np.array(times))
    errors_line.set_ydata(np.array(errors))
    return planned_positions_line, desired_positions_line, actual_positions_line, errors_line


base_topic = sys.argv[1]
g_joint_name = sys.argv[2]

state_topic = base_topic + '/state'
goal_topic = base_topic + '/follow_joint_trajectory/goal'

goals_lock = threading.Lock()
goals = []
controller_states_lock = threading.Lock()
controller_states = []

rospy.init_node('plot_trajectory', anonymous=True)

(planned_positions_line,) = plt.plot([0.0], label='Planned positions')
(desired_positions_line,) = plt.plot([0.0], label='Desired positions')
(actual_positions_line,) = plt.plot([0.0], label='Actual positions')
(errors_line,) = plt.plot([0.0], label='Errors')
t_start = rospy.Time.now().to_sec()
plt.xlim(t_start, t_start + 500.0)
plt.ylim(-np.pi, np.pi)

follow_trajectory_goal_sub = rospy.Subscriber(goal_topic, FollowJointTrajectoryActionGoal, callback=add_goal)

controller_state_sub = rospy.Subscriber(state_topic, JointTrajectoryControllerState, add_controller_state)

plt.xlabel('Time from start (s)')
plt.ylabel('Position (radians)')
plt.legend()
plt.grid(True)
ani = animation.FuncAnimation(plt.gcf(), update, interval=100)
plt.show()
