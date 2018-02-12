/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometric_shapes/solid_primitive_dims.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";

void openGripper(trajectory_msgs::JointTrajectory &posture)
{
  posture.joint_names.resize(1);
  posture.joint_names[0] = "gripper_finger_joint";

  posture.points.resize(1);
  posture.points[0].positions.resize(1);
  posture.points[0].positions[0] = 0.0;

  posture.points[0].time_from_start.fromSec(5.0);
}

void closedGripper(trajectory_msgs::JointTrajectory &posture)
{
  posture.joint_names.resize(1);
  posture.joint_names[0] = "gripper_finger_joint";

  posture.points.resize(1);
  posture.points[0].positions.resize(1);
  posture.points[0].positions[0] = 0.48;   // closed around coke can: 0.48; fully closed: 0.76

  posture.points[0].time_from_start.fromSec(5.0);
}

moveit::planning_interface::MoveItErrorCode pick(moveit::planning_interface::MoveGroupInterface &group)
{
  std::vector<moveit_msgs::Grasp> grasps;

  // --- calculate desired pose of gripper_tcp when grasping
  // pose of coke can
  geometry_msgs::PoseStamped p;
  p.header.frame_id = "base_footprint";
  p.pose.position.x = 1.05;
  p.pose.position.y = -0.25;
  p.pose.position.z = 1.0 + 0.1239 / 2.0;

  // add z offset to grasp coke can a bit higher than center, to get the gripper away from the table
  p.pose.position.z += 0.03;

  p.pose.orientation.x = 0;
  p.pose.orientation.y = 0;
  p.pose.orientation.z = 0;
  p.pose.orientation.w = 1.0;
  moveit_msgs::Grasp g;
  g.grasp_pose = p;

  g.pre_grasp_approach.direction.vector.x = 1.0;
  g.pre_grasp_approach.direction.header.frame_id = "gripper_tcp";
  g.pre_grasp_approach.min_distance = 0.2;
  g.pre_grasp_approach.desired_distance = 0.4;

  g.post_grasp_retreat.direction.header.frame_id = "base_footprint";
  g.post_grasp_retreat.direction.vector.z = 1.0;
  g.post_grasp_retreat.min_distance = 0.1;
  g.post_grasp_retreat.desired_distance = 0.25;

  openGripper(g.pre_grasp_posture);

  closedGripper(g.grasp_posture);

  grasps.push_back(g);
  group.setSupportSurfaceName("table");
  return group.pick("coke_can", grasps);
}

moveit::planning_interface::MoveItErrorCode place(moveit::planning_interface::MoveGroupInterface &group)
{
  std::vector<moveit_msgs::PlaceLocation> loc;

  // --- calculate desired pose of gripper_tcp when placing
  // desired pose of coke can
  geometry_msgs::PoseStamped p;
  p.header.frame_id = "base_footprint";
  p.pose.position.x = 1.05;
  p.pose.position.y = 0.25;  // only difference to grasping: 0.25 instead of -0.25
  p.pose.position.z = 1.0 + 0.1239 / 2.0;

  // add z offset to grasp coke can a bit higher than center, to get the gripper away from the table
  p.pose.position.z += 0.03;

  p.pose.orientation.x = 0;
  p.pose.orientation.y = 0;
  p.pose.orientation.z = 0;
  p.pose.orientation.w = 1.0;
  moveit_msgs::PlaceLocation g;
  g.place_pose = p;

  g.pre_place_approach.direction.vector.z = -1.0;
  g.post_place_retreat.direction.vector.x = -1.0;
  g.post_place_retreat.direction.header.frame_id = "base_footprint";
  g.pre_place_approach.direction.header.frame_id = "gripper_tcp";
  g.pre_place_approach.min_distance = 0.1;
  g.pre_place_approach.desired_distance = 0.2;
  g.post_place_retreat.min_distance = 0.1;
  g.post_place_retreat.desired_distance = 0.25;

  openGripper(g.post_place_posture);

  loc.push_back(g);
  group.setSupportSurfaceName("table");

  // add path constraints
  moveit_msgs::Constraints constr;
  constr.orientation_constraints.resize(1);
  moveit_msgs::OrientationConstraint &ocm = constr.orientation_constraints[0];
  ocm.link_name = "gripper_tcp";
  ocm.header.frame_id = p.header.frame_id;
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.2;
  ocm.absolute_z_axis_tolerance = M_PI;
  ocm.weight = 1.0;
  group.setPathConstraints(constr);
  group.setPlannerId("RRTConnectkConfigDefault");

  auto error_code = group.place("coke_can", loc);
  group.clearPathConstraints();
  return error_code;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mobipick_pick_n_place");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  ros::Publisher pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);

  ros::WallDuration(1.0).sleep();

  moveit::planning_interface::MoveGroupInterface group("arm");
  group.setPlanningTime(45.0);

  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "odom_comb";

  // remove pole
  co.id = "pole";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  // add pole
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.3;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.0;
  co.primitive_poses.resize(1);
  co.primitive_poses[0].position.x = 0.7;
  co.primitive_poses[0].position.y = -0.4;
  co.primitive_poses[0].position.z = 0.85;
  co.primitive_poses[0].orientation.w = 1.0;
  //TODO pub_co.publish(co);

  // remove table
  co.id = "table";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  // add table
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 1.0;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 2.0;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.0;
  co.primitive_poses[0].position.x = 1.0;
  co.primitive_poses[0].position.y = 0.0;
  co.primitive_poses[0].position.z = 1.0 / 2.0;
  pub_co.publish(co);

  co.id = "coke_can";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);

  moveit_msgs::AttachedCollisionObject aco;
  aco.object = co;
  pub_aco.publish(aco);

  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  co.primitives[0].dimensions.resize(
          geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.1239;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.0335;

  co.primitive_poses[0].position.x = 1.05;
  co.primitive_poses[0].position.y = -0.25;
  co.primitive_poses[0].position.z = 1.0 + 0.1239 / 2.0;
  pub_co.publish(co);

  // wait a bit for ros things to initialize
  ros::WallDuration(1.0).sleep();

  // plan to observe the table
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  group.setNamedTarget("observe100cm_front");
  moveit::planning_interface::MoveItErrorCode error_code = group.plan(plan);
  if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    ROS_INFO("Planning to observation pose SUCCESSFUL");
  } else
  {
    ROS_ERROR("Planning to observation pose FAILED");
    return 1;
  }

  // move to observation pose
  error_code = group.move();
  if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    ROS_INFO("Moving to observation pose SUCCESSFUL");
  } else
  {
    ROS_ERROR("Moving to observation pose FAILED");
    return 1;
  }
  ros::WallDuration(2.0).sleep();

  // pick
  error_code = pick(group);
  if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    ROS_INFO("Picking SUCCESSFUL");
  } else
  {
    ROS_ERROR("Picking FAILED");
    return 1;
  }
  ros::WallDuration(1.0).sleep();

  // place
  error_code = place(group);
  if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    ROS_INFO("Placing SUCCESSFUL");
  } else
  {
    ROS_ERROR("Placing FAILED");
    return 1;
  }

  // plan to go home
  group.setNamedTarget("home");
  error_code = group.plan(plan);
  if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    ROS_INFO("Planning to home pose SUCCESSFUL");
  } else
  {
    ROS_ERROR("Planning to home pose FAILED");
    return 1;
  }

  // move to home
  error_code = group.move();
  if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    ROS_INFO("Moving to home pose SUCCESSFUL");
  } else
  {
    ROS_ERROR("Moving to home pose FAILED");
    return 1;
  }
  ros::WallDuration(2.0).sleep();
  ros::waitForShutdown();
  return 0;
}
