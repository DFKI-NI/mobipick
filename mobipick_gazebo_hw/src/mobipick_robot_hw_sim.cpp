// Sim-only hardware interface: DefaultRobotHWSim with an absolute joint position for limited revolute joints (#250).
//
// DefaultRobotHWSim::readSim keeps joint_position_ += shortest_angular_distance(joint_position_, Position(0)) for every
// joint that is not prismatic. If the arm is flung by more than pi between two controller updates (e.g. a Gazebo
// set_model_state teleport of the whole robot), the shortest step is the wrong way round and /joint_states gains or loses a
// whole turn for good (wrist_3 179.9 -> 539.9 -> 15299.9 deg), although Gazebo's own angle is right. A revolute joint with
// position limits (all UR arm joints of the Mobipick) can never legitimately leave its limit range, so its absolute Gazebo
// angle is the state; continuous joints keep the accumulation.
#include <gazebo_ros_control/default_robot_hw_sim.h>

namespace mobipick_gazebo_hw
{

class MobipickRobotHWSim : public gazebo_ros_control::DefaultRobotHWSim
{
public:
  void readSim(ros::Time time, ros::Duration period) override
  {
    gazebo_ros_control::DefaultRobotHWSim::readSim(time, period);   // positions (accumulated), velocity, effort
    for (unsigned int j = 0; j < n_dof_; ++j)
    {
      if (joint_types_[j] == urdf::Joint::REVOLUTE)
      {
#if GAZEBO_MAJOR_VERSION >= 8
        joint_position_[j] = sim_joints_[j]->Position(0);
#else
        joint_position_[j] = sim_joints_[j]->GetAngle(0).Radian();
#endif
      }
    }
  }
};

}  // namespace mobipick_gazebo_hw

PLUGINLIB_EXPORT_CLASS(mobipick_gazebo_hw::MobipickRobotHWSim, gazebo_ros_control::RobotHWSim)
