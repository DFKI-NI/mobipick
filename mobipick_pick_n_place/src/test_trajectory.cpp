#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_trajectory");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  while (ros::ok())
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    std::vector<double> joint_group_positions{ 0.0, 0.0, 0.0, 0.0, 0.0, -3.10 };
    move_group.setJointValueTarget(joint_group_positions);
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO("Planning to joint space goal 1 %s", success ? "" : "FAILED");
    if (success)
      move_group.move();

    joint_group_positions = { 0.0, 0.0, 0.0, 0.0, 0.0, 3.10 };
    move_group.setJointValueTarget(joint_group_positions);
    success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO("Planning to joint space goal 2 %s", success ? "" : "FAILED");
    if (success)
      move_group.move();
  }

  ros::shutdown();
  return 0;
}
