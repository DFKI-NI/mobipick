#pragma once

#include <memory>

#include <boost/shared_ptr.hpp>
#include <string>

#include <actionlib/server/action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_server_goal_handle.h>
#include <ros/ros.h>

#include <mobipick_gripper_effort_controller/gripper_mapping.h>

namespace mobipick_gripper_effort_controller
{

/**
 * GripperCommand action controller for a single actuated gripper joint on the EffortJointInterface.
 *
 * Goals carry the jaw gap in metres (as the Robotiq action server on the real robot expects) and
 * max_effort on the Robotiq 30..100 scale. The controller converts the gap into a finger joint
 * setpoint, runs a PID on the joint position error and clamps the PID output to the joint torque
 * mapped from max_effort. The clamped torque is what Gazebo applies, so a low max_effort really
 * results in a weak grasp and a stalled gripper keeps pressing with at most that torque.
 *
 * Goal validation mirrors robotiq_2f_gripper_action_server: a position outside [min_gap, max_gap]
 * or a max_effort outside [min_effort, max_effort] is refused with the same warning the real server
 * prints, and the gripper does not move. (The real server leaves such a goal without result; here
 * the goal is rejected so clients get a terminal state instead of hanging.)
 *
 * ROS parameters (in the controller namespace):
 *   joint                 (string, required)  actuated joint name
 *   action_name           (string, "")       action namespace, resolved relative to the robot
 *                                             namespace; empty -> controller namespace
 *   gap_open/gap_closed/joint_open/joint_closed   see GapJointMapping
 *   min_gap/max_gap       (double, -0.015/0.14) accepted goal positions [m], same as the real server's args
 *   min_effort/max_effort (double, 30/100)      accepted max_effort range, same as the real server's args
 *   joint_effort_min/joint_effort_max          torque [Nm] applied at min_effort/max_effort, see EffortMapping
 *   hold_max_effort       (double, 100)         effort used to hold the position while no goal is active
 *   max_joint_velocity    (double, 0.5)       the position setpoint is ramped towards the goal at this
 *                                             speed [rad/s]; keep it below the URDF velocity limit
 *   pid/{p,i,d,i_clamp,antiwindup}            PID on the joint position error, output in Nm
 *   goal_tolerance        (double, 0.005)     gap tolerance [m] to report reached_goal
 *   stall_velocity_threshold (double, 0.01)   joint speed [rad/s] below which the joint counts as stalled
 *   stall_timeout         (double, 1.0)       seconds of stall before the goal is finished
 *   settle_time           (double, 0.3)       seconds after the setpoint ramp reached the goal at which the
 *                                             goal is finished regardless of motion (reached or stalled)
 *   stall_is_success      (bool, true)        stalled goals finish as succeeded (stalled=true) instead of aborted
 *   goal_timeout          (double, 10.0)      seconds after which an unfinished goal is treated as stalled (0 = never)
 *   action_monitor_rate   (double, 20.0)      Hz at which feedback is published
 */
class GripperEffortController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  GripperEffortController();
  ~GripperEffortController() override;

  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  using ActionServer = actionlib::ActionServer<control_msgs::GripperCommandAction>;
  using GoalHandle = ActionServer::GoalHandle;
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<control_msgs::GripperCommandAction>;
  using RealtimeGoalHandlePtr = boost::shared_ptr<RealtimeGoalHandle>;

  /// Setpoint written from the action callbacks (non-RT) and read in update() (RT).
  struct Command
  {
    double joint_position = 0.0;  ///< goal [rad]
    double torque_limit = 0.0;    ///< [Nm]
    unsigned long seq = 0;        ///< incremented per goal so update() can restart the setpoint ramp
  };

  void goalCB(GoalHandle gh);
  void cancelCB(GoalHandle gh);
  void preemptActiveGoal();
  void holdCurrentPosition();
  void checkForSuccess(const ros::Time& time, double joint_error, double gap, double velocity, double effort);
  void finishGoal(const RealtimeGoalHandlePtr& active_goal, double gap, double effort, bool reached, const char* reason);
  void publishState(const ros::Time& time, double setpoint, double position, double error, double velocity,
                    double command, double torque_limit);

  std::string name_;
  std::string joint_name_;
  hardware_interface::JointHandle joint_;
  ros::NodeHandle controller_nh_;

  GapJointMapping gap_mapping_;
  EffortMapping effort_mapping_;
  control_toolbox::Pid pid_;

  double goal_tolerance_joint_ = 0.0;
  double max_joint_velocity_ = 0.5;
  double goal_timeout_ = 10.0;
  double stall_velocity_threshold_ = 0.01;
  double stall_timeout_ = 1.0;
  double settle_time_ = 0.3;
  bool stall_is_success_ = true;
  ros::Duration action_monitor_period_;

  realtime_tools::RealtimeBuffer<Command> command_;
  Command command_struct_;  ///< scratch copy used by the non-RT side

  std::unique_ptr<ActionServer> action_server_;
  RealtimeGoalHandlePtr rt_active_goal_;
  control_msgs::GripperCommandResultPtr pre_alloc_result_;
  ros::Timer goal_handle_timer_;
  ros::Time last_movement_time_;
  ros::Time goal_start_time_;

  // RT-side state of the setpoint ramp
  double setpoint_ = 0.0;
  unsigned long setpoint_seq_ = 0;
  bool result_requested_ = false;  ///< a terminal result was handed to the monitor timer for the active goal
  bool ramp_done_ = false;         ///< the setpoint ramp reached the goal position
  ros::Time ramp_done_time_;

  using StatePublisher = realtime_tools::RealtimePublisher<control_msgs::JointControllerState>;
  std::unique_ptr<StatePublisher> state_publisher_;
  ros::Duration state_publish_period_;
  ros::Time last_state_publish_time_;
};

}  // namespace mobipick_gripper_effort_controller
