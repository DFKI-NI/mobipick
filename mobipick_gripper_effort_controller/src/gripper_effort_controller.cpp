#include <mobipick_gripper_effort_controller/gripper_effort_controller.h>

#include <cmath>

#include <pluginlib/class_list_macros.h>

namespace mobipick_gripper_effort_controller
{

namespace
{
std::string controllerName(const ros::NodeHandle& nh)
{
  const std::string ns = nh.getNamespace();
  const std::size_t pos = ns.find_last_of('/');
  return pos == std::string::npos ? ns : ns.substr(pos + 1);
}
}  // namespace

GripperEffortController::GripperEffortController() = default;

GripperEffortController::~GripperEffortController()
{
  // Make sure no callback fires on a half-destroyed object.
  if (action_server_)
  {
    action_server_.reset();
  }
}

bool GripperEffortController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh,
                                   ros::NodeHandle& controller_nh)
{
  controller_nh_ = controller_nh;
  name_ = controllerName(controller_nh);

  if (!controller_nh.getParam("joint", joint_name_))
  {
    ROS_ERROR_NAMED(name_, "No 'joint' parameter given in namespace %s", controller_nh.getNamespace().c_str());
    return false;
  }
  try
  {
    joint_ = hw->getHandle(joint_name_);
  }
  catch (const hardware_interface::HardwareInterfaceException& e)
  {
    ROS_ERROR_NAMED(name_, "Could not claim joint '%s' on the EffortJointInterface: %s", joint_name_.c_str(),
                    e.what());
    return false;
  }

  // Gap <-> joint mapping
  controller_nh.param("gap_open", gap_mapping_.gap_open, gap_mapping_.gap_open);
  controller_nh.param("gap_closed", gap_mapping_.gap_closed, gap_mapping_.gap_closed);
  controller_nh.param("joint_open", gap_mapping_.joint_open, gap_mapping_.joint_open);
  controller_nh.param("joint_closed", gap_mapping_.joint_closed, gap_mapping_.joint_closed);
  // Accepted goal range, same parameter names as robotiq_2f_gripper_action_server
  controller_nh.param("min_gap", gap_mapping_.accept_min_gap, gap_mapping_.accept_min_gap);
  controller_nh.param("max_gap", gap_mapping_.accept_max_gap, gap_mapping_.accept_max_gap);
  if (!gap_mapping_.valid())
  {
    ROS_ERROR_NAMED(name_,
                    "Invalid gap/joint mapping: gap_open=%f gap_closed=%f joint_open=%f joint_closed=%f "
                    "min_gap=%f max_gap=%f",
                    gap_mapping_.gap_open, gap_mapping_.gap_closed, gap_mapping_.joint_open,
                    gap_mapping_.joint_closed, gap_mapping_.accept_min_gap, gap_mapping_.accept_max_gap);
    return false;
  }

  // Effort scale -> joint torque limit mapping; min_effort/max_effort are the real server's parameter names
  controller_nh.param("min_effort", effort_mapping_.input_min, effort_mapping_.input_min);
  controller_nh.param("max_effort", effort_mapping_.input_max, effort_mapping_.input_max);
  controller_nh.param("joint_effort_min", effort_mapping_.torque_min, effort_mapping_.torque_min);
  controller_nh.param("joint_effort_max", effort_mapping_.torque_max, effort_mapping_.torque_max);
  controller_nh.param("hold_max_effort", effort_mapping_.hold_input, effort_mapping_.hold_input);
  if (!effort_mapping_.valid())
  {
    ROS_ERROR_NAMED(name_,
                    "Invalid effort mapping: min_effort=%f max_effort=%f joint_effort_min=%f "
                    "joint_effort_max=%f hold_max_effort=%f",
                    effort_mapping_.input_min, effort_mapping_.input_max, effort_mapping_.torque_min,
                    effort_mapping_.torque_max, effort_mapping_.hold_input);
    return false;
  }

  // PID on the joint position error
  if (!pid_.init(ros::NodeHandle(controller_nh, "pid")))
  {
    ROS_ERROR_NAMED(name_, "Failed to initialise PID gains from namespace %s/pid",
                    controller_nh.getNamespace().c_str());
    return false;
  }

  double goal_tolerance_gap = 0.005;
  controller_nh.param("goal_tolerance", goal_tolerance_gap, goal_tolerance_gap);
  goal_tolerance_joint_ = gap_mapping_.gapToleranceToJoint(goal_tolerance_gap);
  controller_nh.param("stall_velocity_threshold", stall_velocity_threshold_, stall_velocity_threshold_);
  controller_nh.param("stall_timeout", stall_timeout_, stall_timeout_);
  controller_nh.param("stall_is_success", stall_is_success_, stall_is_success_);
  controller_nh.param("goal_timeout", goal_timeout_, goal_timeout_);
  controller_nh.param("settle_time", settle_time_, settle_time_);
  controller_nh.param("max_joint_velocity", max_joint_velocity_, max_joint_velocity_);
  if (!(max_joint_velocity_ > 0.0))
  {
    ROS_ERROR_NAMED(name_, "max_joint_velocity must be positive, got %f", max_joint_velocity_);
    return false;
  }

  double action_monitor_rate = 20.0;
  controller_nh.param("action_monitor_rate", action_monitor_rate, action_monitor_rate);
  action_monitor_period_ = ros::Duration(1.0 / std::max(1e-3, action_monitor_rate));

  double state_publish_rate = 50.0;
  controller_nh.param("state_publish_rate", state_publish_rate, state_publish_rate);
  state_publish_period_ = ros::Duration(1.0 / std::max(1e-3, state_publish_rate));
  state_publisher_.reset(new StatePublisher(controller_nh, "state", 1));
  state_publisher_->msg_.header.frame_id = joint_name_;

  // Action server: either in the controller namespace or in an explicitly named namespace relative to
  // the robot namespace (e.g. "gripper_hw" so that MoveIt and the other clients keep working unchanged).
  std::string action_name;
  controller_nh.param("action_name", action_name, action_name);
  ros::NodeHandle action_nh = action_name.empty() ? controller_nh : ros::NodeHandle(root_nh, action_name);
  action_server_.reset(new ActionServer(action_nh, "", boost::bind(&GripperEffortController::goalCB, this, _1),
                                        boost::bind(&GripperEffortController::cancelCB, this, _1), false));
  action_server_->start();

  pre_alloc_result_.reset(new control_msgs::GripperCommandResult());

  ROS_INFO_NAMED(name_,
                 "Initialised gripper effort controller on joint '%s': action '%s', gap [%.3f, %.3f] m -> joint "
                 "[%.3f, %.3f] rad, max_effort [%.1f, %.1f] -> torque [%.2f, %.2f] Nm",
                 joint_name_.c_str(), action_nh.getNamespace().c_str(), gap_mapping_.gap_closed,
                 gap_mapping_.gap_open, gap_mapping_.joint_closed, gap_mapping_.joint_open,
                 effort_mapping_.input_min, effort_mapping_.input_max, effort_mapping_.torque_min,
                 effort_mapping_.torque_max);
  return true;
}

void GripperEffortController::starting(const ros::Time& time)
{
  pid_.reset();
  holdCurrentPosition();
  last_movement_time_ = time;
  last_state_publish_time_ = time;
}

void GripperEffortController::stopping(const ros::Time& /*time*/)
{
  preemptActiveGoal();
  joint_.setCommand(0.0);
}

void GripperEffortController::holdCurrentPosition()
{
  command_struct_.joint_position = gap_mapping_.clampJoint(joint_.getPosition());
  command_struct_.torque_limit = effort_mapping_.toTorqueLimit(effort_mapping_.hold_input);
  ++command_struct_.seq;
  command_.writeFromNonRT(command_struct_);
}

void GripperEffortController::update(const ros::Time& time, const ros::Duration& period)
{
  const Command cmd = *command_.readFromRT();

  const double position = joint_.getPosition();
  const double velocity = joint_.getVelocity();

  // Ramp the tracked setpoint towards the goal at max_joint_velocity. A new goal restarts the ramp
  // from the current position so the PID never sees a large step; on contact the ramp keeps going,
  // the error grows and the output saturates at the torque limit, which is the grasp force.
  if (cmd.seq != setpoint_seq_)
  {
    setpoint_seq_ = cmd.seq;
    setpoint_ = position;
    result_requested_ = false;
    ramp_done_ = false;
    pid_.reset();
  }
  setpoint_ = stepTowards(setpoint_, cmd.joint_position, max_joint_velocity_ * period.toSec());
  if (!ramp_done_ && setpoint_ == cmd.joint_position)
  {
    ramp_done_ = true;
    ramp_done_time_ = time;
  }

  const double tracking_error = setpoint_ - position;
  const double raw = pid_.computeCommand(tracking_error, -velocity, period);
  const double effort = clampTorque(raw, cmd.torque_limit);
  joint_.setCommand(effort);

  const double gap = gap_mapping_.jointToGap(position);
  checkForSuccess(time, cmd.joint_position - position, gap, velocity, effort);
  publishState(time, setpoint_, position, tracking_error, velocity, effort, cmd.torque_limit);
}

void GripperEffortController::checkForSuccess(const ros::Time& time, double joint_error, double gap,
                                              double velocity, double effort)
{
  RealtimeGoalHandlePtr active_goal(rt_active_goal_);
  if (!active_goal)
  {
    return;
  }

  // The monitor timer (action_monitor_rate) turns the request below into the actual action result,
  // so the goal stays ACTIVE for a few cycles after the request; do not re-evaluate it meanwhile.
  if (result_requested_ || active_goal->gh_.getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE)
  {
    return;
  }

  // Feedback is filled here and sent by the non-RT timer.
  active_goal->preallocated_feedback_->position = gap;
  active_goal->preallocated_feedback_->effort = effort;
  active_goal->preallocated_feedback_->reached_goal = false;
  active_goal->preallocated_feedback_->stalled = false;
  active_goal->setFeedback(active_goal->preallocated_feedback_);

  ROS_DEBUG_THROTTLE_NAMED(1.0, name_,
                          "goal active: joint %.4f rad (setpoint %.4f, goal error %.4f, tol %.4f), vel %.4f rad/s, "
                          "effort %.3f Nm, status %d",
                          joint_.getPosition(), setpoint_, joint_error, goal_tolerance_joint_, velocity, effort,
                          active_goal->gh_.getGoalStatus().status);

  const bool within_tolerance = std::fabs(joint_error) < goal_tolerance_joint_;
  if (within_tolerance)
  {
    finishGoal(active_goal, gap, effort, true, "goal reached");
    return;
  }

  // Not at the goal. Three ways to finish: the joint stopped moving (blocked by an object), the
  // setpoint ramp finished and the fingers had settle_time to settle (they are pressing on something
  // that keeps them slightly moving, e.g. a light object), or the overall goal timeout expired.
  const bool timed_out = goal_timeout_ > 0.0 && (time - goal_start_time_).toSec() > goal_timeout_;
  const bool settled = ramp_done_ && (time - ramp_done_time_).toSec() > settle_time_;
  if (std::fabs(velocity) > stall_velocity_threshold_)
  {
    last_movement_time_ = time;
  }
  const bool stalled = (time - last_movement_time_).toSec() > stall_timeout_;

  if (timed_out)
  {
    finishGoal(active_goal, gap, effort, false, "goal timeout");
  }
  else if (stalled)
  {
    finishGoal(active_goal, gap, effort, false, "gripper stalled");
  }
  else if (settled)
  {
    finishGoal(active_goal, gap, effort, false, "ramp finished and settle time elapsed");
  }
}

void GripperEffortController::finishGoal(const RealtimeGoalHandlePtr& active_goal, double gap, double effort,
                                         bool reached, const char* reason)
{
  pre_alloc_result_->position = gap;
  pre_alloc_result_->effort = effort;
  pre_alloc_result_->reached_goal = reached;
  pre_alloc_result_->stalled = !reached;
  const bool success = reached || stall_is_success_;
  ROS_INFO_NAMED(name_, "%s: gap %.4f m, effort %.3f Nm; requesting %s%s", reason, gap, effort,
                 success ? "SUCCEEDED" : "ABORTED", reached ? "" : " (stalled=true)");
  // Do not reset rt_active_goal_ here: the monitor timer only tracks the handle weakly and is the
  // one delivering the result to the client. The handle stays until the next goal or a cancel.
  if (success)
  {
    active_goal->setSucceeded(pre_alloc_result_);
  }
  else
  {
    active_goal->setAborted(pre_alloc_result_);
  }
  result_requested_ = true;
}

void GripperEffortController::publishState(const ros::Time& time, double setpoint, double position, double error,
                                           double velocity, double command, double torque_limit)
{
  if (!state_publisher_ || (time - last_state_publish_time_) < state_publish_period_)
  {
    return;
  }
  if (state_publisher_->trylock())
  {
    last_state_publish_time_ = time;
    auto& msg = state_publisher_->msg_;
    msg.header.stamp = time;
    msg.set_point = setpoint;
    msg.process_value = position;
    msg.process_value_dot = velocity;
    msg.error = error;
    msg.time_step = 0.0;
    msg.command = command;
    // The remaining fields carry the active limits so they can be plotted alongside the command.
    msg.p = torque_limit;
    msg.i = 0.0;
    msg.d = 0.0;
    msg.i_clamp = torque_limit;
    msg.antiwindup = false;
    state_publisher_->unlockAndPublish();
  }
}

void GripperEffortController::goalCB(GoalHandle gh)
{
  if (!this->isRunning())
  {
    ROS_ERROR_NAMED(name_, "Can't accept a new gripper goal, the controller is not running");
    control_msgs::GripperCommandResult result;
    gh.setRejected(result);
    return;
  }

  const auto& command = gh.getGoal()->command;
  if (!std::isfinite(command.position) || !std::isfinite(command.max_effort))
  {
    ROS_ERROR_NAMED(name_, "Rejecting gripper goal with non-finite position or effort");
    control_msgs::GripperCommandResult result;
    gh.setRejected(result);
    return;
  }

  // Same checks (and warnings) as robotiq_2f_gripper_action_server::goalToRegisterState on the real
  // robot. There the goal is accepted, the warning printed and no command is issued to the gripper;
  // here the goal is rejected so the client gets a terminal state, but the gripper does not move either.
  if (!gap_mapping_.accepted(command.position))
  {
    ROS_WARN_NAMED(name_, "Goal gripper gap size is out of range(%f to %f): %f m", gap_mapping_.accept_min_gap,
                   gap_mapping_.accept_max_gap, command.position);
    ROS_INFO_NAMED(name_, "%s No goal issued to gripper", name_.c_str());
    control_msgs::GripperCommandResult result;
    result.position = gap_mapping_.jointToGap(joint_.getPosition());
    gh.setRejected(result);
    return;
  }
  if (!effort_mapping_.accepted(command.max_effort))
  {
    ROS_WARN_NAMED(name_, "Goal gripper effort out of range (%f to %f N): %f N", effort_mapping_.input_min,
                   effort_mapping_.input_max, command.max_effort);
    ROS_INFO_NAMED(name_, "%s No goal issued to gripper", name_.c_str());
    control_msgs::GripperCommandResult result;
    result.position = gap_mapping_.jointToGap(joint_.getPosition());
    gh.setRejected(result);
    return;
  }

  preemptActiveGoal();
  gh.setAccepted();

  command_struct_.joint_position = gap_mapping_.gapToJoint(command.position);
  command_struct_.torque_limit = effort_mapping_.toTorqueLimit(command.max_effort);
  ++command_struct_.seq;
  command_.writeFromNonRT(command_struct_);

  ROS_INFO_NAMED(name_, "New gripper goal: gap %.4f m (joint %.4f rad), max_effort %.1f -> torque limit %.2f Nm",
                 command.position, command_struct_.joint_position, command.max_effort,
                 command_struct_.torque_limit);

  pre_alloc_result_->reached_goal = false;
  pre_alloc_result_->stalled = false;
  last_movement_time_ = ros::Time::now();
  goal_start_time_ = last_movement_time_;

  RealtimeGoalHandlePtr rt_goal(new RealtimeGoalHandle(gh));
  goal_handle_timer_ = controller_nh_.createTimer(action_monitor_period_, &RealtimeGoalHandle::runNonRealtime, rt_goal);
  goal_handle_timer_.start();
  rt_active_goal_ = rt_goal;
  ROS_INFO_NAMED(name_, "goal %s accepted (handle valid: %d, status %d), monitor timer at %.1f Hz on %s",
                 gh.getGoalID().id.c_str(), rt_goal->valid() ? 1 : 0, gh.getGoalStatus().status,
                 1.0 / action_monitor_period_.toSec(), controller_nh_.getNamespace().c_str());
}

void GripperEffortController::cancelCB(GoalHandle gh)
{
  RealtimeGoalHandlePtr active_goal(rt_active_goal_);
  ROS_INFO_NAMED(name_, "cancel requested for goal %s (active goal: %s)", gh.getGoalID().id.c_str(),
                 active_goal ? active_goal->gh_.getGoalID().id.c_str() : "none");
  if (active_goal && active_goal->gh_ == gh)
  {
    rt_active_goal_.reset();
    // Keep holding wherever the fingers are right now, with the default torque.
    holdCurrentPosition();
    active_goal->gh_.setCanceled();
  }
}

void GripperEffortController::preemptActiveGoal()
{
  RealtimeGoalHandlePtr active_goal(rt_active_goal_);
  if (active_goal)
  {
    rt_active_goal_.reset();
    if (active_goal->gh_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
    {
      active_goal->gh_.setCanceled();
    }
  }
}

}  // namespace mobipick_gripper_effort_controller

PLUGINLIB_EXPORT_CLASS(mobipick_gripper_effort_controller::GripperEffortController, controller_interface::ControllerBase)
