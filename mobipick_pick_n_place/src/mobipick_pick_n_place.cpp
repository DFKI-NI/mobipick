/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  Copyright (c) 2018, DFKI GmbH
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

/* Authors: Ioan Sucan, Martin Günther */

#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/solid_primitive_dims.h>

#include <vision_msgs/Detection3DArray.h>
#include "mobipick_pick_n_place/fake_object_recognition.h"

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
  posture.points[0].positions[0] = 0.65;   // closed around power drill: 0.65; fully closed: 0.76

  posture.points[0].time_from_start.fromSec(5.0);
}

moveit::planning_interface::MoveItErrorCode pick(moveit::planning_interface::MoveGroupInterface &group)
{
  std::vector<moveit_msgs::Grasp> grasps;

  // --- calculate desired pose of gripper_tcp when grasping
  // pose of power drill
  geometry_msgs::PoseStamped p;
  p.header.frame_id = "power_drill";
  p.pose.position.x = 0.04;
  p.pose.position.y = 0.0;
  p.pose.position.z = 0.0;

  // // pitch = pi/8
  // p.pose.orientation.x = 0;
  // p.pose.orientation.y = 0.19509032;
  // p.pose.orientation.z = 0;
  // p.pose.orientation.w = 0.98078528;

  // inverse quaternion of the power drill: rpy = (pi / 2, 0, pi) = quat (0, 0.707, 0.707, 0) />
  p.pose.orientation.x = 0.0;
  p.pose.orientation.y = -0.707106781;
  p.pose.orientation.z = -0.707106781;
  p.pose.orientation.w = 0.0;

  moveit_msgs::Grasp g;
  g.grasp_pose = p;
  g.grasp_quality = 1.0;

  g.pre_grasp_approach.direction.vector.x = 1.0;
  g.pre_grasp_approach.direction.header.frame_id = "gripper_tcp";
  g.pre_grasp_approach.min_distance = 0.1;
  g.pre_grasp_approach.desired_distance = 0.3;

  g.post_grasp_retreat.direction.header.frame_id = "base_footprint";
  g.post_grasp_retreat.direction.vector.z = 1.0;
  g.post_grasp_retreat.min_distance = 0.1;
  g.post_grasp_retreat.desired_distance = 0.25;

  openGripper(g.pre_grasp_posture);

  closedGripper(g.grasp_posture);

  grasps.push_back(g);

  // Add a second grasp in case the first doesn't work
  p.pose.orientation.x = 0;
  p.pose.orientation.y = 0;
  p.pose.orientation.z = 0;
  p.pose.orientation.w = 1;
  g.grasp_pose = p;
  g.grasp_quality = 0.5;

  grasps.push_back(g);

  group.setSupportSurfaceName("table");
  return group.pick("power_drill", grasps);
}

moveit::planning_interface::MoveItErrorCode place(moveit::planning_interface::MoveGroupInterface &group)
{
  std::vector<moveit_msgs::PlaceLocation> loc;

  // --- calculate desired pose of gripper_tcp when placing
  // desired pose of power drill
  geometry_msgs::PoseStamped p;
  p.header.frame_id = "base_footprint";
  p.pose.position.x = 0.90;
  p.pose.position.y = 0.20;  // only difference to grasping: 0.20 instead of -0.20
  p.pose.position.z = 1.10;

  // quaternion of the power drill: rpy = (pi / 2, 0, pi) = quat (0, 0.707, 0.707, 0) />
  p.pose.orientation.x = 0.0;
  p.pose.orientation.y = 0.707106781;
  p.pose.orientation.z = 0.707106781;
  p.pose.orientation.w = 0.0;
  moveit_msgs::PlaceLocation g;
  g.place_pose = p;

  g.pre_place_approach.direction.header.frame_id = "base_footprint";
  g.pre_place_approach.desired_distance = 0.2;
  g.pre_place_approach.direction.vector.z = -1.0;
  g.pre_place_approach.min_distance = 0.1;
  g.post_place_retreat.direction.header.frame_id = "gripper_tcp";
  g.post_place_retreat.direction.vector.x = -1.0;
  g.post_place_retreat.desired_distance = 0.25;
  g.post_place_retreat.min_distance = 0.1;

  openGripper(g.post_place_posture);

  loc.push_back(g);
  group.setSupportSurfaceName("table");

  // add path constraints - doesn't work for place :(
  //  moveit_msgs::Constraints constr;
  //  constr.orientation_constraints.resize(1);
  //  moveit_msgs::OrientationConstraint &ocm = constr.orientation_constraints[0];
  //  ocm.link_name = "gripper_tcp";
  //  ocm.header.frame_id = p.header.frame_id;
  //  ocm.orientation.x = 0.0;
  //  ocm.orientation.y = 0.0;
  //  ocm.orientation.z = 0.0;
  //  ocm.orientation.w = 1.0;
  //  ocm.absolute_x_axis_tolerance = 0.2;
  //  ocm.absolute_y_axis_tolerance = 0.2;
  //  ocm.absolute_z_axis_tolerance = M_PI;
  //  ocm.weight = 1.0;
  //  group.setPathConstraints(constr);

  auto error_code = group.place("power_drill", loc);
  group.clearPathConstraints();
  return error_code;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mobipick_pick_n_place");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;

  moveit::planning_interface::MoveGroupInterface group("arm");
  group.setPlanningTime(45.0);
  group.setPlannerId("RRTConnectkConfigDefault");

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  /* ********************* UPDATE PLANNING SCENE ********************* */

  // get objects from object detection
  vision_msgs::Detection3DArrayConstPtr detections = ros::topic::waitForMessage<vision_msgs::Detection3DArray>(
          "detected_objects", nh, ros::Duration(10.0));
  if (!detections)
  {
    ROS_ERROR("Timed out while waiting for a message on topic detected_objects!");
    return 1;
  }

  // add objects to planning scene
  bool found_table = false;
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  for (auto &&det3d : detections->detections)
  {
    if (det3d.results.empty())
    {
      ROS_ERROR("Detections3D message has empty results!");
      return 1;
    }

    if (det3d.results[0].id == ObjectID::TABLE)
      found_table = true;

    moveit_msgs::CollisionObject co;
    co.header = det3d.header;
    co.id = id_to_string(det3d.results[0].id);
    co.operation = moveit_msgs::CollisionObject::ADD;
    co.primitives.resize(1);
    co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = det3d.bbox.size.x + 0.04;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = det3d.bbox.size.y + 0.04;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = det3d.bbox.size.z + 0.04;
    co.primitive_poses.resize(1);
    co.primitive_poses[0] = det3d.bbox.center;

    collision_objects.push_back(co);
  }
  if (!found_table)
  {
    // Add table from MRK Lab
    moveit_msgs::CollisionObject co;
    co.header.stamp = detections->header.stamp;
    co.header.frame_id = "map";
    co.id = id_to_string(ObjectID::TABLE);
    co.operation = moveit_msgs::CollisionObject::ADD;
    co.primitives.resize(1);
    co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.70;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.60;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.93;
    co.primitive_poses.resize(1);
    co.primitive_poses[0].position.x = 10.05;
    co.primitive_poses[0].position.y = 11.50;
    co.primitive_poses[0].position.z = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] / 2.0;
    co.primitive_poses[0].orientation.w = 1.0;

    collision_objects.push_back(co);
  }

  planning_scene_interface.applyCollisionObjects(collision_objects);

  // detach all objects
  auto attached_objects = planning_scene_interface.getAttachedObjects();
  for (auto &&object : attached_objects)
  {
    group.detachObject(object.first);
  }

  /* ********************* PLAN AND EXECUTE MOVES ********************* */

  // plan to observe the table
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  group.setNamedTarget("observe100cm_right");
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
  return 0;
}
