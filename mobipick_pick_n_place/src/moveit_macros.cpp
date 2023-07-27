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

/* Authors: Ioan Sucan, Martin Günther, Alexander Sung */

#include "mobipick_pick_n_place/fake_object_recognition.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <mobipick_pick_n_place/MoveItMacroAction.h>

// MoveIt!
#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <actionlib/client/simple_action_client.h>

// gripper
#include <control_msgs/GripperCommandAction.h>

#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <vision_msgs/Detection3DArray.h>

#include <eigen_conversions/eigen_msg.h>

#include <sstream>

namespace mobipick
{
std::string tf_prefix_ = "mobipick";

struct GrapsPoseDefine
{
  Eigen::Isometry3d grasp_pose;
  std::float_t gripper_width;
};

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(1);
  posture.joint_names[0] = tf_prefix_ + "gripper_finger_joint";

  posture.points.resize(1);
  posture.points[0].positions.resize(1);
  posture.points[0].positions[0] = 0.1;

  posture.points[0].effort.resize(1);
  posture.points[0].effort[0] = 30;
  posture.points[0].time_from_start.fromSec(5.0);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture, std::float_t gripper_width = 0.63)
{
  posture.joint_names.resize(1);
  posture.joint_names[0] = tf_prefix_ + "gripper_finger_joint";

  posture.points.resize(1);
  posture.points[0].positions.resize(1);
  posture.points[0].positions[0] =
      gripper_width;  // closed around power drill: 0.65; fully closed: 0.76  TODO: should be 0.42 for top grasp

  posture.points[0].effort.resize(1);
  posture.points[0].effort[0] = 80;
  posture.points[0].time_from_start.fromSec(5.0);
}

moveit::core::MoveItErrorCode pick(moveit::planning_interface::MoveGroupInterface& group)
{
  std::vector<moveit_msgs::Grasp> grasps;

  // --- calculate grasps
  // this is using standard frame orientation: x forward, y left, z up, relative to object bounding box center

  std::vector<GrapsPoseDefine> grasp_poses;
  /*
  {
    // GRASP 1: pitch = pi/8  (grasp handle from upper back)
    GrapsPoseDefine grasp_pose_define;

    Eigen::AngleAxisd rotation = Eigen::AngleAxisd(M_PI/8, Eigen::Vector3d(0.0d, 1.0d, 0.0d));
    grasp_pose_define.grasp_pose = Eigen::Isometry3d::Identity();
    grasp_pose_define.grasp_pose.translate(Eigen::Vector3d(-0.05d, 0.0d, 0.01d));
    grasp_pose_define.grasp_pose.rotate(rotation);
    grasp_pose_define.gripper_width=0.03;
    grasp_poses.push_back(grasp_pose_define);

  }
  */
  {
    // GRASP 2: pitch = pi/2 (grasp top part from above)
    GrapsPoseDefine grasp_pose_define;
    grasp_pose_define.grasp_pose = Eigen::Isometry3d::Identity();
    grasp_pose_define.grasp_pose.translate(Eigen::Vector3d(-0.03d, 0.0d, 0.085d));
    grasp_pose_define.grasp_pose.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0.0d, 1.0d, 0.0d)));
    grasp_pose_define.gripper_width = 0.03;
    grasp_poses.push_back(grasp_pose_define);
  }
  // commented out because if used, move to transport will fail
  /*
  {
    // GRASP 3: pitch = pi/2 (grasp top part from above mirrored)
    GrapsPoseDefine grasp_pose_define;
    grasp_pose_define.grasp_pose = Eigen::Isometry3d::Identity();
    grasp_pose_define.grasp_pose.translate(Eigen::Vector3d(-0.03d, 0.0d, 0.085d));
    grasp_pose_define.grasp_pose.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0.0d, 1.0d, 0.0d)));
    grasp_pose_define.grasp_pose.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(1.0d, 0.0d, 0.0d)));
    grasp_pose_define.gripper_width = 0.03;
    grasp_poses.push_back(grasp_pose_define);
  }
  */

  for (auto&& grasp_pose : grasp_poses)
  {
    // rotate grasp pose from CAD model orientation to standard orientation (x forward, y left, z up)
    // Eigen quaternion = wxyz, not xyzw
    Eigen::Isometry3d bbox_center_rotated = Eigen::Isometry3d::Identity();
    bbox_center_rotated.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0.0d, 0.0d, 1.0d)));

    bbox_center_rotated.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d(1.0d, 0.0d, 0.0d)));

    // --- calculate desired pose of gripper_tcp (in power drill frame) when grasping
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "power_drill";
    tf::poseEigenToMsg(bbox_center_rotated * grasp_pose.grasp_pose, p.pose);
    ROS_DEBUG_STREAM("Grasp pose:\n" << p.pose);

    moveit_msgs::Grasp g;

    g.grasp_pose = p;
    g.grasp_quality = 1.0;
    ROS_INFO_STREAM("Grasp pose:\n" << p.pose);

    g.pre_grasp_approach.direction.vector.x = 1.0;
    g.pre_grasp_approach.direction.header.frame_id = tf_prefix_ + "gripper_tcp";
    g.pre_grasp_approach.min_distance = 0.08;
    g.pre_grasp_approach.desired_distance = 0.25;

    g.post_grasp_retreat.direction.header.frame_id = tf_prefix_ + "base_link";
    g.post_grasp_retreat.direction.vector.z = 1.0;
    g.post_grasp_retreat.min_distance = 0.1;
    g.post_grasp_retreat.desired_distance = 0.15;

    openGripper(g.pre_grasp_posture);

    closedGripper(g.grasp_posture, grasp_pose.gripper_width);

    grasps.push_back(g);
  }

  group.setSupportSurfaceName("table");
  return group.pick("power_drill", grasps);
}

moveit::core::MoveItErrorCode place(moveit::planning_interface::MoveGroupInterface& group)
{
  std::vector<moveit_msgs::PlaceLocation> loc;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // get table height
  std::vector<std::string> object_ids;
  object_ids.push_back("table");
  auto table = planning_scene_interface.getObjects(object_ids).at("table");
  double table_height = table.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z];
  ROS_INFO("Table height: %f", table_height);

  // --- calculate desired pose of power drill (in base_link frame) when placing
  geometry_msgs::PoseStamped p;

  Eigen::Isometry3d place_pose = Eigen::Isometry3d::Identity();
  place_pose.translate(Eigen::Vector3d(-0.0d, -0.9d, table_height + 0.11d));
  place_pose.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d(1.0d, 0.0d, 0.0d)));
  place_pose.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d(0.0d, 1.0d, 0.0d)));
  p.header.frame_id = tf_prefix_ + "base_link";
  tf::poseEigenToMsg(place_pose, p.pose);

  moveit_msgs::PlaceLocation g;
  g.place_pose = p;
  g.allowed_touch_objects.push_back("table");

  g.pre_place_approach.direction.header.frame_id = tf_prefix_ + "base_link";
  g.pre_place_approach.desired_distance = 0.2;
  g.pre_place_approach.direction.vector.z = -1.0;
  g.pre_place_approach.min_distance = 0.1;
  g.post_place_retreat.direction.header.frame_id = tf_prefix_ + "gripper_tcp";
  g.post_place_retreat.direction.vector.x = -1.0;
  g.post_place_retreat.desired_distance = 0.25;
  g.post_place_retreat.min_distance = 0.1;

  openGripper(g.post_place_posture);

  loc.push_back(g);
  group.setSupportSurfaceName("table");

  // add path constraints - doesn't work for place :(

  ROS_INFO_STREAM("Place at " << g.place_pose);
  auto error_code = group.place("power_drill", loc);
  group.clearPathConstraints();
  return error_code;
}

void setOrientationContraints(moveit::planning_interface::MoveGroupInterface& group, double factor_pi = 0.3)
{
  moveit_msgs::Constraints constr;

  // TODO: don't recreate tf_listener all the time
  tf::TransformListener tf_listener;
  tf::StampedTransform transform;

  try
  {
    tf_listener.waitForTransform(tf_prefix_ + "gripper_tcp", tf_prefix_ + "base_link", ros::Time(0), ros::Duration(1.0));
    tf_listener.lookupTransform(tf_prefix_ + "gripper_tcp", tf_prefix_ + "base_link", ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ROS_INFO("Transformation not found!");
  }
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = tf_prefix_ + "gripper_tcp";
  ocm.header.frame_id = tf_prefix_ + "base_link";

  ocm.orientation.x = -0.5;
  ocm.orientation.y = 0.5;
  ocm.orientation.z = -0.5;
  ocm.orientation.w = -0.5;
  ocm.absolute_x_axis_tolerance = factor_pi * M_PI;
  ocm.absolute_y_axis_tolerance = factor_pi * M_PI;
  ocm.absolute_z_axis_tolerance = 2.0 * M_PI;
  ocm.weight = 0.8;

  constr.orientation_constraints.push_back(ocm);

  group.setPathConstraints(constr);
}

int updatePlanningScene(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                        ros::NodeHandle& nh, moveit::planning_interface::MoveGroupInterface& group)
{
  // get objects from object detection
  bool found_power_drill = false;
  uint visionCounter = 0;
  while (!found_power_drill)
  {
    if (!ros::ok())
      return 0;

    vision_msgs::Detection3DArrayConstPtr detections =
        ros::topic::waitForMessage<vision_msgs::Detection3DArray>("dope/detected_objects", nh, ros::Duration(30.0));
    if (!detections)
    {
      ROS_ERROR("Timed out while waiting for a message on topic detected_objects!");
      return 1;
    }

    // add objects to planning scener
    bool found_table = false;
    bool found_roof = false;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    for (auto&& det3d : detections->detections)
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
      else if (det3d.results[0].id == ObjectID::ROOF)
        found_roof = true;

      moveit_msgs::CollisionObject co;
      co.header = detections->header;
      co.id = id_to_string(det3d.results[0].id);
      co.operation = moveit_msgs::CollisionObject::ADD;
      co.primitives.resize(1);
      co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
      co.primitives[0].dimensions.resize(
          geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
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
      co.header.frame_id = "/map";
      co.id = id_to_string(ObjectID::TABLE);
      co.operation = moveit_msgs::CollisionObject::ADD;
      co.primitives.resize(1);
      co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
      co.primitives[0].dimensions.resize(
          geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.80;
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.80;
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.73;
      co.primitive_poses.resize(1);
      co.primitive_poses[0].position.x = 12.3;
      co.primitive_poses[0].position.y = 3.8;
      co.primitive_poses[0].position.z = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] / 2.0;
      co.primitive_poses[0].orientation.w = 1.0;

      collision_objects.push_back(co);
    }

    if (!found_roof)
    {
      // Add roof from Lab
      moveit_msgs::CollisionObject co;
      co.header.stamp = detections->header.stamp;
      co.header.frame_id = "/map";
      co.id = id_to_string(ObjectID::ROOF);
      co.operation = moveit_msgs::CollisionObject::ADD;
      co.primitives.resize(1);
      co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
      co.primitives[0].dimensions.resize(
          geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 5.0;
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 5.0;
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.2;
      co.primitive_poses.resize(1);
      co.primitive_poses[0].position.x = 12.3;
      co.primitive_poses[0].position.y = 3.8;
      co.primitive_poses[0].position.z = co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] / 2.0 + 2.2;
      co.primitive_poses[0].orientation.w = 1.0;

      collision_objects.push_back(co);
    }

    if (!found_power_drill)
      ROS_INFO_THROTTLE(1.0, "Still waiting for power drill...");

    planning_scene_interface.applyCollisionObjects(collision_objects);
  }

  // detach all objects
  auto attached_objects = planning_scene_interface.getAttachedObjects();
  for (auto&& object : attached_objects)
  {
    group.detachObject(object.first);
  }
  return 1;
}

moveit::core::MoveItErrorCode move(moveit::planning_interface::MoveGroupInterface& group, double dx = 0.0,
                                   double dy = 0.0, double dz = 0.0, double droll = 0.0, double dpitch = 0.0,
                                   double dyaw = 0.0)
{
  robot_state::RobotState start_state(*group.getCurrentState());
  group.setStartState(start_state);

  Eigen::Isometry3d pose;
  geometry_msgs::PoseStamped current_pose = group.getCurrentPose();
  geometry_msgs::PoseStamped target_pose = current_pose;
  tf::poseMsgToEigen(current_pose.pose, pose);
  pose.translate(Eigen::Vector3d(dx, dy, dz));
  pose.rotate(Eigen::AngleAxisd(dyaw, Eigen::Vector3d(0.0, 0.0, 1.0)));
  pose.rotate(Eigen::AngleAxisd(dpitch, Eigen::Vector3d(0.0, 1.0, 0.0)));
  pose.rotate(Eigen::AngleAxisd(droll, Eigen::Vector3d(1.0, 0.0, 0.0)));
  tf::poseEigenToMsg(pose, target_pose.pose);
  ROS_INFO_STREAM("Target pose frame: " << target_pose.header.frame_id);
  group.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  auto error_code = group.plan(my_plan);
  bool success = (error_code == moveit::core::MoveItErrorCode::SUCCESS);

  ROS_INFO("Move planning (pose goal) %s", success ? "" : "FAILED");
  if (success)
  {
    error_code = group.execute(my_plan);
  }
  return error_code;
}

moveit::core::MoveItErrorCode moveToCartPose(moveit::planning_interface::MoveGroupInterface& group,
                                             Eigen::Isometry3d cartesian_pose,
                                             std::string base_frame = tf_prefix_ + "ur5_base_link",
                                             std::string target_frame = tf_prefix_ + "gripper_tcp")
{
  robot_state::RobotState start_state(*group.getCurrentState());
  group.setStartState(start_state);

  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = base_frame;
  tf::poseEigenToMsg(cartesian_pose, target_pose.pose);

  group.setPoseTarget(target_pose, target_frame);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  auto error_code = group.plan(my_plan);
  bool success = (error_code == moveit::core::MoveItErrorCode::SUCCESS);

  ROS_INFO("Move planning (pose goal) %s", success ? "" : "FAILED");
  if (success)
  {
    error_code = group.execute(my_plan);
  }
  return error_code;
}

std::unique_ptr<moveit::planning_interface::MoveGroupInterface> group_ptr;
std::unique_ptr<moveit::planning_interface::MoveGroupInterface::Plan> plan_ptr;
std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_ptr;
std::unique_ptr<actionlib::SimpleActionClient<control_msgs::GripperCommandAction>> gripper_ac_ptr;
}  // namespace mobipick

using namespace mobipick;

class MoveItMacroAction
{
protected:
  typedef bool (MoveItMacroAction::*moveit_function_t)();
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<mobipick_pick_n_place::MoveItMacroAction> as_;
  std::map<std::string, moveit_function_t> moveit_functions;
  std::string action_name_;
  mobipick_pick_n_place::MoveItMacroResult result_;

public:
  MoveItMacroAction(std::string name)
    : as_(nh_, name, boost::bind(&MoveItMacroAction::executeCallback, this, _1), false), action_name_(name)
  {
    // GRIPPER
    gripper_ac_ptr =
        std::make_unique<actionlib::SimpleActionClient<control_msgs::GripperCommandAction>>("gripper_hw", true);

    // wait for the gripper action server to come up
    while (!gripper_ac_ptr->waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the gripper action server to come up");
    }
    ROS_INFO("Connected to gripper action server");
    // ARM
    group_ptr = std::make_unique<moveit::planning_interface::MoveGroupInterface>("arm");
    group_ptr->setPlanningTime(45.0);
    group_ptr->setPlannerId("RRTConnect");
    // MOVE IT
    plan_ptr = std::make_unique<moveit::planning_interface::MoveGroupInterface::Plan>();
    // PLANNING INTERFACE
    planning_scene_interface_ptr = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();

    registerMoveItFunctions();
    as_.start();
  }

  ~MoveItMacroAction(void)
  {
  }

  bool hasAttachedObjects()
  {
    return !planning_scene_interface_ptr->getAttachedObjects().empty();
  }

  bool moveArmToTarget(const std::string& target_name)
  {
    /* ******************* MOVE ARM TO TARGET ****************************** */
    // plan
    group_ptr->setPlannerId("RRTConnect");
    group_ptr->setStartStateToCurrentState();
    group_ptr->setNamedTarget(target_name);
    moveit::core::MoveItErrorCode error_code = group_ptr->plan(*plan_ptr);
    if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
    {
      ROS_INFO_STREAM("Planning to " << target_name << " pose SUCCESSFUL");
    }
    else
    {
      ROS_ERROR_STREAM("Planning to " << target_name << " pose FAILED");
      return false;
    }

    // move
    error_code = group_ptr->execute(*plan_ptr);
    if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
    {
      ROS_INFO_STREAM("Moving to " << target_name << " pose SUCCESSFUL");
    }
    else
    {
      ROS_ERROR_STREAM("Moving to " << target_name << " pose FAILED");
      return false;
    }

    return true;
  }

  bool captureObject()
  {
    /* ********************* PLAN AND EXECUTE MOVES ********************* */

    // plan to observe the table
    group_ptr->setNamedTarget("observe100cm_right");
    moveit::core::MoveItErrorCode error_code = group_ptr->plan(*plan_ptr);
    if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
    {
      ROS_INFO("Planning to observation pose SUCCESSFUL");
    }
    else
    {
      ROS_ERROR("Planning to observation pose FAILED");
      return false;
    }

    // move to observation pose
    error_code = group_ptr->execute(*plan_ptr);
    if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
    {
      ROS_INFO("Moving to observation pose SUCCESSFUL");
    }
    else
    {
      ROS_ERROR("Moving to observation pose FAILED");
      return false;
    }

    ros::WallDuration(5.0).sleep();
    return true;
  }

  bool pickUpObject()
  {
    /* ********************* PICK ********************* */

    // clear octomap
    ros::ServiceClient clear_octomap = nh_.serviceClient<std_srvs::Empty>("clear_octomap");
    std_srvs::Empty srv;
    group_ptr->clearPathConstraints();
    // pick
    moveit::core::MoveItErrorCode error_code;
    uint pickPlanAttempts = 0;
    bool success = true;
    do
    {
      clear_octomap.call(srv);

      updatePlanningScene(*planning_scene_interface_ptr, nh_, *group_ptr);
      error_code = pick(*group_ptr);
      ++pickPlanAttempts;

      if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
      {
        ROS_INFO("Picking SUCCESSFUL");
      }
      else if ((error_code == moveit::core::MoveItErrorCode::PLANNING_FAILED ||
                error_code == moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN) &&
               pickPlanAttempts < 10)
      {
        ROS_INFO("Planning for Picking FAILED");
      }
      else
      {
        ROS_ERROR("Picking FAILED");
        success = false;
      }

    } while ((error_code == moveit::core::MoveItErrorCode::PLANNING_FAILED ||
              error_code == moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN) &&
             pickPlanAttempts < 10);
    return success;
  }

  bool moveArmToHandover()
  {
    /* ********************* PLAN AND EXECUTE TO HAND OVER POSE ********************* */
    setOrientationContraints(*group_ptr, 0.666666);
    Eigen::Isometry3d hand_over_pose = Eigen::Isometry3d::Identity();
    hand_over_pose.translate(Eigen::Vector3d(0.8, -0.4, 0.2));
    hand_over_pose.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0.0, 1.0, 0.0)));
    hand_over_pose.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(1.0, 0.0, 0.0)));
    hand_over_pose.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0.0, 0.0, 1.0)));

    if (moveToCartPose(*group_ptr, hand_over_pose) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      ROS_INFO("Move arm to HANDOVER succeeded");
    }
    else
    {
      ROS_INFO("Move arm to HANDOVER failed");
      return false;
    }

    return true;
  }

  bool releaseGripper()
  {
    control_msgs::GripperCommandGoal gripper_goal;

    gripper_goal.command.position = 0.1;
    gripper_goal.command.max_effort = 30.0;
    gripper_ac_ptr->sendGoal(gripper_goal);
    gripper_ac_ptr->waitForResult();

    if (gripper_ac_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Gripper move SUCCESSFUL, detach all Objects");

      // detach all objects
      auto attached_objects = planning_scene_interface_ptr->getAttachedObjects();
      std::vector<std::string> objects_to_remove;
      for (auto&& object : attached_objects)
      {
        ROS_INFO_STREAM("Detach object " << object.first);
        group_ptr->detachObject(object.first);
        objects_to_remove.push_back(object.first);
      }
      planning_scene_interface_ptr->removeCollisionObjects(objects_to_remove);
      group_ptr->clearPathConstraints();
    }
    else
    {
      ROS_INFO("Gripper move FAILED");
      return false;
    }

    return true;
  }

  bool placeObject()
  {
    /* ********************* PLACE ********************* */

    // move(group, -0.05, -0.05, 0.2);
    ros::WallDuration(1.0).sleep();
    group_ptr->setPlannerId("RRTConnect");
    ROS_INFO("Start Placing");
    // place
    uint placePlanAttempts = 0;
    moveit::core::MoveItErrorCode error_code;
    bool success = true;
    do
    {
      group_ptr->setPlanningTime(40 + 10 * placePlanAttempts);
      setOrientationContraints(*group_ptr, 0.3);
      error_code = place(*group_ptr);
      ++placePlanAttempts;
      if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
      {
        ROS_INFO("Placing SUCCESSFUL");
      }
      else if ((error_code == moveit::core::MoveItErrorCode::PLANNING_FAILED ||
                error_code == moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN ||
                error_code == moveit::core::MoveItErrorCode::TIMED_OUT) &&
               placePlanAttempts < 10)
      {
        ROS_INFO("Planning for Placing FAILED");
        ros::WallDuration(1.0).sleep();
        // move(group, 0.01, 0.01, -0.01); //TODO: make a random/suitable move
      }
      else
      {
        ROS_ERROR("Placing FAILED");
        success = false;
      }
    } while ((error_code == moveit::core::MoveItErrorCode::PLANNING_FAILED ||
              error_code == moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN ||
              error_code == moveit::core::MoveItErrorCode::TIMED_OUT) &&
             placePlanAttempts < 10);
    return success;
  }

  void registerMoveItFunctions()
  {
    moveit_functions["HasAttachedObjects"] = &MoveItMacroAction::hasAttachedObjects;
    moveit_functions["CaptureObject"] = &MoveItMacroAction::captureObject;
    moveit_functions["PickUpObject"] = &MoveItMacroAction::pickUpObject;
    moveit_functions["MoveArmToHandover"] = &MoveItMacroAction::moveArmToHandover;
    moveit_functions["ReleaseGripper"] = &MoveItMacroAction::releaseGripper;
    moveit_functions["PlaceObject"] = &MoveItMacroAction::placeObject;
  }

  void executeCallback(const mobipick_pick_n_place::MoveItMacroGoalConstPtr& goal_ptr)
  {
    if (goal_ptr->type == "target")
    {
      result_.result = moveArmToTarget(goal_ptr->name);
    }
    else if (goal_ptr->type == "function")
    {
      std::map<std::string, moveit_function_t>::iterator it = moveit_functions.find(goal_ptr->name);
      if (it != moveit_functions.end())
      {
        moveit_function_t function = it->second;
        result_.result = (this->*function)();
      }
      else
      {
        ROS_ERROR_STREAM("Invalid function name '" << goal_ptr->name << "' in moveit macro.");
        result_.result = false;
      }
    }
    else
    {
      ROS_ERROR_STREAM("Invalid type '" << goal_ptr->type << "' used in moveit macro.");
      result_.result = false;
    }
    if (result_.result)
    {
      as_.setSucceeded(result_);
    }
    else
    {
      // Note: There is no setFailed() method. Aborted is the default value used when no result is set.
      as_.setAborted(result_);
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_macros");

  ros::NodeHandle nh_priv("~");
  std::string param_path;
  if (nh_priv.searchParam("tf_prefix", param_path))
    nh_priv.getParam(param_path, tf_prefix_);
  nh_priv.param<std::string>("tf_prefix", tf_prefix_, "mobipick");

  // ensure tf_prefix_ ends with exactly 1 '/' if nonempty, or "" if empty
  tf_prefix_ = tf_prefix_.erase(tf_prefix_.find_last_not_of('/') + 1) + "/";
  if (tf_prefix_.length() == 1)
    tf_prefix_ = "";

  MoveItMacroAction moveit_macros("moveit_macros");

  ros::spin();
  return 0;
}
