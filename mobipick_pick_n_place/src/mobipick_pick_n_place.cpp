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
#include <eigen_conversions/eigen_msg.h>

#include "std_msgs/String.h"
#include <sstream>

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

  // --- calculate grasps
  // this is using standard frame orientation: x forward, y left, z up, relative to object bounding box center
  std::vector<Eigen::Affine3d> grasp_poses;

  {
    // GRASP 1: pitch = pi/8  (grasp handle from upper back)
    Eigen::AngleAxisd rotation = Eigen::AngleAxisd(M_PI/8, Eigen::Vector3d(0.0d, 1.0d, 0.0d));
    Eigen::Affine3d grasp_pose = Eigen::Affine3d::Identity();
    grasp_pose.translate(Eigen::Vector3d(-0.05d, 0.0d, 0.0d));
    grasp_pose.rotate(rotation);
    grasp_poses.push_back(grasp_pose);
  }

  {
    // GRASP 2: pitch = 0 (grasp handle horizontally)
    Eigen::AngleAxisd rotation = Eigen::AngleAxisd(M_PI/8, Eigen::Vector3d(0.0d, 1.0d, 0.0d));
    Eigen::Affine3d grasp_pose = Eigen::Affine3d::Identity();
    grasp_pose.translate(Eigen::Vector3d(-0.05d, 0.0d, 0.0d));
    grasp_pose.rotate(rotation);
    grasp_poses.push_back(grasp_pose);
  }

  {
    // GRASP 3: pitch = pi/2 (grasp top part from above)
    Eigen::AngleAxisd rotation = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0.0d, 1.0d, 0.0d));
    Eigen::Affine3d grasp_pose = Eigen::Affine3d::Identity();
    grasp_pose.translate(Eigen::Vector3d(-0.03d, 0.0d, 0.08d));
    grasp_pose.rotate(rotation);
    grasp_poses.push_back(grasp_pose);
  }


  for (auto&& grasp_pose : grasp_poses)
  {
    // rotate grasp pose from CAD model orientation to standard orientation (x forward, y left, z up)
    // inverse quaternion of the power drill: rpy = (pi / 2, 0, pi) = quat (0, 0.707, 0.707, 0)
    // Eigen quaternion = wxyz, not xyzw
    Eigen::Affine3d bbox_center_rotated; // = Eigen::Affine3d::Identity();

    geometry_msgs::Pose power_drill;
    power_drill.orientation.x = 0.0;
    power_drill.orientation.y = -0.707106781;
    power_drill.orientation.z = -0.707106781;
    power_drill.orientation.w = 0.0;

    power_drill.position.x = 0.00;
    power_drill.position.y = 0.00;
    power_drill.position.z = 0.00;

    tf::poseMsgToEigen(power_drill, bbox_center_rotated);
    //bbox_center_rotated.rotate(Eigen::Quaterniond(0.0d, 0.0d, -0.707106781d, -0.707106781d));

    // --- calculate desired pose of gripper_tcp when grasping
    // pose of power drill
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "power_drill";
    tf::poseEigenToMsg(bbox_center_rotated * grasp_pose, p.pose);
    ROS_DEBUG_STREAM("Grasp pose:\n" << p.pose);

    moveit_msgs::Grasp g;
    // p.pose.orientation.x = 0.0;
    // p.pose.orientation.y = -0.707106781;
    // p.pose.orientation.z = -0.707106781;
    // p.pose.orientation.w = 0.0;

    g.grasp_pose = p;
    g.grasp_quality = 1.0;
    ROS_INFO_STREAM("Grasp pose:\n" << p.pose);

    g.pre_grasp_approach.direction.vector.x = 1.0;
    g.pre_grasp_approach.direction.header.frame_id = "gripper_tcp";
    g.pre_grasp_approach.min_distance = 0.08;
    g.pre_grasp_approach.desired_distance = 0.25;

    g.post_grasp_retreat.direction.header.frame_id = "base_link";
    g.post_grasp_retreat.direction.vector.z = 1.0;
    g.post_grasp_retreat.min_distance = 0.1;
    g.post_grasp_retreat.desired_distance = 0.15;

    openGripper(g.pre_grasp_posture);

    closedGripper(g.grasp_posture);

    grasps.push_back(g);
  }

  group.setSupportSurfaceName("table");
  return group.pick("power_drill", grasps);
}

moveit::planning_interface::MoveItErrorCode place(moveit::planning_interface::MoveGroupInterface &group)
{
  std::vector<moveit_msgs::PlaceLocation> loc;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // get table height
  std::vector<std::string> object_ids;
  object_ids.push_back("table");
  auto table = planning_scene_interface.getObjects(object_ids).at("table");
  double table_height =
      table.primitive_poses[0].position.z + table.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] / 2.0;
  ROS_INFO("Table height: %f", table_height);

  // --- calculate desired pose of gripper_tcp when placing
  // desired pose of power drill
  geometry_msgs::PoseStamped p;
  p.header.frame_id = "base_footprint";
  p.pose.position.x = -0.142;
  p.pose.position.y = -0.969;
  p.pose.position.z = 1.137 ; //table_height + 0.10;   // power drill center (with large battery pack) is about 0.10 m above table

  p.pose.orientation.x = 0.5;
  p.pose.orientation.y = 0.5;
  p.pose.orientation.z = 0.5;
  p.pose.orientation.w = 0.5;
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

  ros::Publisher pubGripper = nh.advertise<std_msgs::String>("/mobipick/gripper_control", 1);

  std_msgs::String msgGripper;
  std::stringstream ssGripper;


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
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  /* ********************* UPDATE PLANNING SCENE ********************* */

  // get objects from object detection

  bool found_power_drill = false;
  uint visionCounter = 0;
  while (!found_power_drill)
  {
    if (!ros::ok())
      return 0;

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
      else if (det3d.results[0].id == ObjectID::POWER_DRILL)
        found_power_drill = true;

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
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.70;
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.94;
      co.primitive_poses.resize(1);
      co.primitive_poses[0].position.x = 10.05;
      co.primitive_poses[0].position.y = 11.35;
      co.primitive_poses[0].position.z = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] / 2.0;
      co.primitive_poses[0].orientation.w = 1.0;

      collision_objects.push_back(co);
    }

    if (!found_power_drill)
      ROS_INFO_THROTTLE(1.0, "Still waiting for power drill...");

    planning_scene_interface.applyCollisionObjects(collision_objects);
  }

  // detach all objects
  auto attached_objects = planning_scene_interface.getAttachedObjects();
  for (auto &&object : attached_objects)
  {
    group.detachObject(object.first);
  }
  // geometry_msgs::PoseStamped p;
  // p.header.frame_id = "base_footprint";
  // p.pose.position.x = -0.142;
  // p.pose.position.y = -0.969;
  // p.pose.position.z =  1.137;
  //
  // // inverse quaternion of the power drill: rpy = (pi / 2, 0, pi) = quat (0, 0.707, 0.707, 0) />
  // p.pose.orientation.x = 0.078;
  // p.pose.orientation.y = -0.038;
  // p.pose.orientation.z = 0.913;
  // p.pose.orientation.w = -0.399;
  //
  // group.setPoseTarget(p, "gripper_tcp");
  // error_code = group.plan(plan);
  // if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  // {
  //   ROS_INFO("Planning to pick1 pose SUCCESSFUL");
  // } else
  // {
  //   ROS_ERROR("Planning to pick1 pose FAILED");
  //   return 1;
  // }
  // // move to observation pose
  // error_code = group.move();
  // if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  // {
  //   ROS_INFO("Moving to pick1 pose SUCCESSFUL");
  // } else
  // {
  //   ROS_ERROR("Moving to pick1 pose FAILED");
  //   return 1;
  // }
  //pick
  uint pickPlanAttempts = 0;
  do {
    error_code = pick(group);
    ++pickPlanAttempts;
    if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      ROS_INFO("Picking SUCCESSFUL");
    }
    else if((error_code == moveit::planning_interface::MoveItErrorCode::PLANNING_FAILED ||error_code == moveit::planning_interface::MoveItErrorCode::INVALID_MOTION_PLAN) && pickPlanAttempts <3  )
    {
      ROS_INFO("Planning for Picking FAILED");
    }
    else
    {
      ROS_ERROR("Picking FAILED");
      return 1;
    }


  } while((error_code == moveit::planning_interface::MoveItErrorCode::PLANNING_FAILED ||error_code == moveit::planning_interface::MoveItErrorCode::INVALID_MOTION_PLAN ) && pickPlanAttempts <3 );

  ros::WallDuration(1.0).sleep();

  //place
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
