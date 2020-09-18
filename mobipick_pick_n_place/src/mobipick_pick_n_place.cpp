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

#include "mobipick_pick_n_place/fake_object_recognition.h"

#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometric_shapes/solid_primitive_dims.h>

#include <vision_msgs/Detection3DArray.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <eigen_conversions/eigen_msg.h>

#include <sstream>

struct GrapsPoseDefine {
    Eigen::Affine3d grasp_pose;
    std::float_t gripper_width;
  };

void openGripper(trajectory_msgs::JointTrajectory &posture)
{
  posture.joint_names.resize(1);
  posture.joint_names[0] = "mobipick/gripper_finger_joint";

  posture.points.resize(1);
  posture.points[0].positions.resize(1);
  posture.points[0].positions[0] = 0.1;

  posture.points[0].effort.resize(1);
  posture.points[0].effort[0]=30;
  posture.points[0].time_from_start.fromSec(5.0);
}

void closedGripper(trajectory_msgs::JointTrajectory &posture, std::float_t gripper_width = 0.63)
{
  posture.joint_names.resize(1);
  posture.joint_names[0] = "mobipick/gripper_finger_joint";

  posture.points.resize(1);
  posture.points[0].positions.resize(1);
  posture.points[0].positions[0] = gripper_width;   // closed around power drill: 0.65; fully closed: 0.76  TODO: should be 0.42 for top grasp

  posture.points[0].effort.resize(1);
  posture.points[0].effort[0]=80;
  posture.points[0].time_from_start.fromSec(5.0);
}

moveit::planning_interface::MoveItErrorCode pick(moveit::planning_interface::MoveGroupInterface &group)
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
    grasp_pose_define.grasp_pose = Eigen::Affine3d::Identity();
    grasp_pose_define.grasp_pose.translate(Eigen::Vector3d(-0.05d, 0.0d, 0.01d));
    grasp_pose_define.grasp_pose.rotate(rotation);
    grasp_pose_define.gripper_width=0.03;
    grasp_poses.push_back(grasp_pose_define);
    
  }

  {
    // GRASP 2: pitch = 0 (grasp handle horizontally)
    GrapsPoseDefine grasp_pose_define;
    grasp_pose_define.grasp_pose = Eigen::Affine3d::Identity();
    grasp_pose_define.grasp_pose.translate(Eigen::Vector3d(-0.065d, 0.0d, 0.02d));
    grasp_pose_define.gripper_width=0.03;
    grasp_poses.push_back(grasp_pose_define);
  }
*/
  {
    // GRASP 3: pitch = pi/2 (grasp top part from above)
    GrapsPoseDefine grasp_pose_define;
    Eigen::AngleAxisd rotation = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0.0d, 1.0d, 0.0d));
    grasp_pose_define.grasp_pose = Eigen::Affine3d::Identity();
    grasp_pose_define.grasp_pose.translate(Eigen::Vector3d(-0.03d, 0.0d, 0.085d));
    grasp_pose_define.grasp_pose.rotate(rotation);
    grasp_pose_define.gripper_width=0.045;
    grasp_poses.push_back(grasp_pose_define);
  }


  for (auto&& grasp_pose : grasp_poses)
  {
    // rotate grasp pose from CAD model orientation to standard orientation (x forward, y left, z up)
    // Eigen quaternion = wxyz, not xyzw
    Eigen::Affine3d bbox_center_rotated= Eigen::Affine3d::Identity();
    bbox_center_rotated.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0.0d, 0.0d, 1.0d)));
    
    bbox_center_rotated.rotate(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(1.0d, 0.0d, 0.0d)));
    
    
    // --- calculate desired pose of gripper_tcp when grasping
    // pose of power drill
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "power_drill";
    tf::poseEigenToMsg(bbox_center_rotated * grasp_pose.grasp_pose, p.pose);
    ROS_DEBUG_STREAM("Grasp pose:\n" << p.pose);

    moveit_msgs::Grasp g;

    g.grasp_pose = p;
    g.grasp_quality = 1.0;
    ROS_INFO_STREAM("Grasp pose:\n" << p.pose);

    g.pre_grasp_approach.direction.vector.x = 1.0;
    g.pre_grasp_approach.direction.header.frame_id = "mobipick/gripper_tcp";
    g.pre_grasp_approach.min_distance = 0.08;
    g.pre_grasp_approach.desired_distance = 0.25;

    g.post_grasp_retreat.direction.header.frame_id = "mobipick/base_link";
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



  Eigen::Affine3d place_pose = Eigen::Affine3d::Identity();
  place_pose.translate(Eigen::Vector3d(-0.0d, -0.7d, table_height + 0.13d));
  place_pose.rotate(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(1.0d, 0.0d, 0.0d)));
  place_pose.rotate(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(0.0d, 1.0d, 0.0d)));
  p.header.frame_id = "mobipick/base_link";
  tf::poseEigenToMsg(place_pose, p.pose);
  
  moveit_msgs::PlaceLocation g;
  g.place_pose = p;
  g.allowed_touch_objects.push_back("table");

  g.pre_place_approach.direction.header.frame_id = "mobipick/base_link";
  g.pre_place_approach.desired_distance = 0.2;
  g.pre_place_approach.direction.vector.z = -1.0;
  g.pre_place_approach.min_distance = 0.1;
  g.post_place_retreat.direction.header.frame_id = "mobipick/gripper_tcp";
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


void setOrientationContraints(moveit::planning_interface::MoveGroupInterface &group)
{
  moveit_msgs::Constraints constr;
  
  tf::TransformListener tf_listener_;
  tf::StampedTransform transform;

  ros::Duration(1).sleep();
  try
  {
    tf_listener_.lookupTransform("mobipick/gripper_tcp", "mobipick/base_link", ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ROS_INFO("Transformation not found!");
  }
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "mobipick/gripper_tcp";
  ocm.header.frame_id = "mobipick/base_link";

  ocm.orientation.x = -0.5;
  ocm.orientation.y = 0.5;
  ocm.orientation.z = -0.5;
  ocm.orientation.w = -0.5;
  ocm.absolute_x_axis_tolerance = M_PI/6;
  ocm.absolute_y_axis_tolerance = M_PI/6;
  ocm.absolute_z_axis_tolerance = 2.0 * M_PI;
  ocm.weight = 0.8;
  

  
  constr.orientation_constraints.push_back(ocm);

  group.setPathConstraints(constr); 
}


int updatePlanningScene(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, ros::NodeHandle &nh, moveit::planning_interface::MoveGroupInterface &group)
{

  // get objects from object detection
  bool found_power_drill = false;
  uint visionCounter = 0;
  while (!found_power_drill)
  {
    if (!ros::ok())
      return 0;

      vision_msgs::Detection3DArrayConstPtr detections = ros::topic::waitForMessage<vision_msgs::Detection3DArray>(
              "/mobipick/dope/detected_objects", nh, ros::Duration(30.0));
      if (!detections)
      {
        ROS_ERROR("Timed out while waiting for a message on topic detected_objects!");
        return 1;
      }



    // add objects to planning scener
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
      co.header = detections->header;
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
      co.header.frame_id = "/mobipick/odom_comb";
      co.id = id_to_string(ObjectID::TABLE);
      co.operation = moveit_msgs::CollisionObject::ADD;
      co.primitives.resize(1);
      co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
      co.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.70;
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.70;
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.73;
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
  return 1;
}

moveit::planning_interface::MoveItErrorCode move(moveit::planning_interface::MoveGroupInterface &group, double dx= 0.0, double dy=0.2, double dz= 0.2)
{

  robot_state::RobotState start_state(*group.getCurrentState());
  group.setStartState(start_state);

  geometry_msgs::PoseStamped actual_pose = group.getCurrentPose();
  geometry_msgs::Pose target_pose = actual_pose.pose;
  target_pose.position.x += dx;
  target_pose.position.y += dy;
  target_pose.position.z += dz;
  ROS_INFO_STREAM("Actual Pose frame: " << actual_pose.header.frame_id);
  group.setPoseTarget(target_pose);


  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO( "Move planning (pose goal) %s", success ? "" : "FAILED");

  auto error_code = group.move();
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
  group.setPlannerId("RRTConnect");
  // NOTE: You can replace this by "RRTstar". This will have the following effects:
  //   - RRT* is an (asymptotically) optimal planner, so the path with minimum path length
  //     will be returned, which leads to much nicer trajectories without unnecessary reconfigurations.
  //   - RRT* will always use the full planning time (see setPlanningTime()), even if an initial
  //     plan is found earlier. If it runs out of time before finding an initial plan, planning will fail.
  //   - Typical run times: RRT* between 1 and 30 seconds, RRTConnect 0.03 seconds
  //   - also see: http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/ompl_interface/ompl_interface_tutorial.html
  //   - also see: https://ompl.kavrakilab.org/planners.html

  ros::Publisher pubGripper = nh.advertise<std_msgs::String>("/mobipick/gripper_control", 1);

  std_msgs::String msgGripper;
  std::stringstream ssGripper;
  
  //ros::ServiceClient attachSrv = nh.serviceClient<mobipick_pick_n_place::AnchoringSrv>("anchoring/attach_power_drill");
  

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
  ros::WallDuration(5.0).sleep();

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  /* ********************* PICK ********************* */

  // clear octomap
  ros::ServiceClient clear_octomap = nh.serviceClient<std_srvs::Empty>("clear_octomap");
  std_srvs::Empty srv;
  
  //pick


  uint pickPlanAttempts = 0;
   do {

    clear_octomap.call(srv);

    updatePlanningScene(planning_scene_interface, nh, group);
    error_code = pick(group);
    ++pickPlanAttempts;



    if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      ROS_INFO("Picking SUCCESSFUL");
    }
    else if((error_code == moveit::planning_interface::MoveItErrorCode::PLANNING_FAILED ||error_code == moveit::planning_interface::MoveItErrorCode::INVALID_MOTION_PLAN) && pickPlanAttempts <10  )
    {
      ROS_INFO("Planning for Picking FAILED");
    }
    else
    {
      ROS_ERROR("Picking FAILED");
      return 1;
    }


  } while((error_code == moveit::planning_interface::MoveItErrorCode::PLANNING_FAILED ||error_code == moveit::planning_interface::MoveItErrorCode::INVALID_MOTION_PLAN ) && pickPlanAttempts <10 );



  /* ********************* PLACE ********************* */

 
  //move(group, -0.05, -0.05, 0.2);
  ros::WallDuration(1.0).sleep();
  group.setPlannerId("PRMstar");
  group.setPlanningTime(30);
  ROS_INFO("Start Placing");
  //place
  uint placePlanAttempts = 0;
  do{
    setOrientationContraints(group);
    error_code = place(group);
    ++placePlanAttempts;
    if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      ROS_INFO("Placing SUCCESSFUL");
    }
    else if((error_code == moveit::planning_interface::MoveItErrorCode::PLANNING_FAILED ||error_code == moveit::planning_interface::MoveItErrorCode::INVALID_MOTION_PLAN || moveit::planning_interface::MoveItErrorCode::TIMED_OUT) && placePlanAttempts <10 )
    {
      ROS_INFO("Planning for Placing FAILED");
      ros::WallDuration(1.0).sleep();
      //move(group, 0.01, 0.01, -0.01); //TODO: make a random/suitable move
    }
    else
    {
      ROS_ERROR("Placing FAILED");
      return 1;
    }
  }while((error_code == moveit::planning_interface::MoveItErrorCode::PLANNING_FAILED ||error_code == moveit::planning_interface::MoveItErrorCode::INVALID_MOTION_PLAN || error_code == moveit::planning_interface::MoveItErrorCode::TIMED_OUT ) && placePlanAttempts <10 );
  
  
  // plan to go home


  group.setPlannerId("RRTConnect");
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