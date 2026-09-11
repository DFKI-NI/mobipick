// Pure, ROS-free helpers used by the GripperEffortController.
// Kept header-only and dependency-free so they can be unit tested without a ROS runtime.
#pragma once

#include <algorithm>
#include <cmath>

namespace mobipick_gripper_effort_controller
{

/// Linear mapping between the gripper jaw gap (metres, what the GripperCommand action and the
/// Robotiq hardware driver use) and the actuated finger joint angle (radians).
struct GapJointMapping
{
  double gap_open = 0.14;      ///< jaw gap when the gripper is fully open [m]
  double gap_closed = 0.0;     ///< jaw gap when the gripper is fully closed [m]
  double joint_open = 0.0;     ///< finger joint position when fully open [rad]
  double joint_closed = 0.755; ///< finger joint position when fully closed [rad]

  /// Range of goal positions that is accepted at all. Mirrors the min_gap/max_gap parameters of the
  /// robotiq_2f_gripper_action_server on the real robot (mobipick_bringup_control.launch uses
  /// -0.015..0.140 m): a goal outside this range is refused instead of executed. Requests below
  /// gap_closed are executed as "fully closed", like the Robotiq position register saturates.
  double accept_min_gap = -0.015;
  double accept_max_gap = 0.14;

  bool valid() const
  {
    return std::isfinite(gap_open) && std::isfinite(gap_closed) && std::isfinite(joint_open) &&
           std::isfinite(joint_closed) && std::isfinite(accept_min_gap) && std::isfinite(accept_max_gap) &&
           gap_open != gap_closed && joint_open != joint_closed && accept_max_gap > accept_min_gap;
  }

  /// True when the real Robotiq action server would accept this goal position.
  bool accepted(double gap) const
  {
    return std::isfinite(gap) && gap >= accept_min_gap && gap <= accept_max_gap;
  }

  double clampGap(double gap) const
  {
    const double lo = std::min(gap_open, gap_closed);
    const double hi = std::max(gap_open, gap_closed);
    return std::min(hi, std::max(lo, gap));
  }

  double clampJoint(double joint) const
  {
    const double lo = std::min(joint_open, joint_closed);
    const double hi = std::max(joint_open, joint_closed);
    return std::min(hi, std::max(lo, joint));
  }

  /// Gap [m] -> joint position [rad]; the gap is clamped to the configured range first.
  double gapToJoint(double gap) const
  {
    const double t = (clampGap(gap) - gap_open) / (gap_closed - gap_open);  // 0 = open, 1 = closed
    return joint_open + t * (joint_closed - joint_open);
  }

  /// Joint position [rad] -> gap [m]; not clamped so that small overshoots stay visible in feedback.
  double jointToGap(double joint) const
  {
    const double t = (joint - joint_open) / (joint_closed - joint_open);
    return gap_open + t * (gap_closed - gap_open);
  }

  /// Gap tolerance [m] expressed as joint tolerance [rad].
  double gapToleranceToJoint(double gap_tolerance) const
  {
    return std::fabs(gap_tolerance * (joint_closed - joint_open) / (gap_closed - gap_open));
  }
};

/// Linear mapping between the effort scale used on the GripperCommand action (the Robotiq action
/// server on the real robot accepts min_effort..max_effort, 30..100 by default) and the joint torque
/// limit applied in simulation [Nm].
struct EffortMapping
{
  double input_min = 30.0;    ///< smallest accepted max_effort on the action (real server: min_effort)
  double input_max = 100.0;   ///< largest accepted max_effort on the action (real server: max_effort)
  double torque_min = 0.1;    ///< joint torque limit at input_min [Nm]
  double torque_max = 0.6;    ///< joint torque limit at input_max [Nm]
  double hold_input = 100.0;  ///< effort (on the input scale) used to hold the position when no goal is active

  bool valid() const
  {
    return std::isfinite(input_min) && std::isfinite(input_max) && std::isfinite(torque_min) &&
           std::isfinite(torque_max) && std::isfinite(hold_input) && input_max > input_min &&
           torque_min >= 0.0 && torque_max >= torque_min;
  }

  /// True when the real Robotiq action server would accept this max_effort. Like on the real robot,
  /// 0 ("do not limit" in control_msgs) is NOT accepted: the Robotiq force register needs a value.
  bool accepted(double max_effort) const
  {
    return std::isfinite(max_effort) && max_effort >= input_min && max_effort <= input_max;
  }

  /// GripperCommand max_effort -> joint torque limit [Nm]. Callers check accepted() first; values
  /// outside the range are clamped so the hold torque can never exceed torque_max.
  double toTorqueLimit(double max_effort) const
  {
    const double e = std::min(input_max, std::max(input_min, max_effort));
    const double t = (e - input_min) / (input_max - input_min);
    return torque_min + t * (torque_max - torque_min);
  }
};

/// Move `current` towards `target` by at most `max_step` (used to ramp the position setpoint so
/// the PID tracks a velocity-limited profile instead of slamming towards a far goal).
inline double stepTowards(double current, double target, double max_step)
{
  max_step = std::fabs(max_step);
  const double delta = target - current;
  if (std::fabs(delta) <= max_step)
  {
    return target;
  }
  return current + (delta > 0.0 ? max_step : -max_step);
}

/// Symmetric saturation of a torque command.
inline double clampTorque(double command, double limit)
{
  limit = std::fabs(limit);
  return std::min(limit, std::max(-limit, command));
}

}  // namespace mobipick_gripper_effort_controller
