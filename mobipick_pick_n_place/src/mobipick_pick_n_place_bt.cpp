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

// Boost
#include <boost/algorithm/string.hpp>

// MoveIt!
#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// Move base
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

// ft observer
#include <mobipick_pick_n_place/FtObserverAction.h>

// gripper
#include <control_msgs/GripperCommandAction.h>

#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <vision_msgs/Detection3DArray.h>

#include <eigen_conversions/eigen_msg.h>

#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <sstream>

namespace mobipick
{

  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

  struct GrapsPoseDefine
  {
    Eigen::Isometry3d grasp_pose;
    std::float_t gripper_width;
  };

  bool paused = true;
  bool failed = false;
  bool pause_active = false;

  void openGripper(trajectory_msgs::JointTrajectory &posture)
  {
    posture.joint_names.resize(1);
    posture.joint_names[0] = "mobipick/gripper_finger_joint";

    posture.points.resize(1);
    posture.points[0].positions.resize(1);
    posture.points[0].positions[0] = 0.1;

    posture.points[0].effort.resize(1);
    posture.points[0].effort[0] = 30;
    posture.points[0].time_from_start.fromSec(5.0);
  }

  void closedGripper(trajectory_msgs::JointTrajectory &posture, std::float_t gripper_width = 0.63)
  {
    posture.joint_names.resize(1);
    posture.joint_names[0] = "mobipick/gripper_finger_joint";

    posture.points.resize(1);
    posture.points[0].positions.resize(1);
    posture.points[0].positions[0] =
        gripper_width; // closed around power drill: 0.65; fully closed: 0.76  TODO: should be 0.42 for top grasp

    posture.points[0].effort.resize(1);
    posture.points[0].effort[0] = 80;
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
      Eigen::AngleAxisd rotation = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0.0d, 1.0d, 0.0d));
      grasp_pose_define.grasp_pose = Eigen::Isometry3d::Identity();
      grasp_pose_define.grasp_pose.translate(Eigen::Vector3d(-0.03d, 0.0d, 0.085d));
      grasp_pose_define.grasp_pose.rotate(rotation);
      grasp_pose_define.gripper_width = 0.03; //
      grasp_poses.push_back(grasp_pose_define);
    }
    /*
    {
      // GRASP 3: pitch = pi/2 (grasp top part from above mirrored)
      GrapsPoseDefine grasp_pose_define;
      grasp_pose_define.grasp_pose = Eigen::Isometry3d::Identity();
      grasp_pose_define.grasp_pose.translate(Eigen::Vector3d(-0.03d, 0.0d, 0.085d));
      grasp_pose_define.grasp_pose.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0.0d, 1.0d, 0.0d)));
      grasp_pose_define.grasp_pose.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(1.0d, 0.0d, 0.0d)));
      grasp_pose_define.gripper_width=0.03; //
      grasp_poses.push_back(grasp_pose_define);
    }
  */

    for (auto &&grasp_pose : grasp_poses)
    {
      // rotate grasp pose from CAD model orientation to standard orientation (x forward, y left, z up)
      // Eigen quaternion = wxyz, not xyzw
      Eigen::Isometry3d bbox_center_rotated = Eigen::Isometry3d::Identity();
      bbox_center_rotated.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0.0d, 0.0d, 1.0d)));

      bbox_center_rotated.rotate(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d(1.0d, 0.0d, 0.0d)));

      // --- calculate desired pose of gripper_tcp when grasping
      // pose of power drill
      geometry_msgs::PoseStamped p;
      p.header.frame_id = "power_drill";
      tf::poseEigenToMsg(bbox_center_rotated * grasp_pose.grasp_pose, p.pose);
      ROS_DEBUG_STREAM("Grasp pose:\n"
                       << p.pose);

      moveit_msgs::Grasp g;

      g.grasp_pose = p;
      g.grasp_quality = 1.0;
      ROS_INFO_STREAM("Grasp pose:\n"
                      << p.pose);

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

    Eigen::Isometry3d place_pose = Eigen::Isometry3d::Identity();
    place_pose.translate(Eigen::Vector3d(-0.0d, -0.9d, table_height + 0.11d));
    place_pose.rotate(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d(1.0d, 0.0d, 0.0d)));
    place_pose.rotate(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d(0.0d, 1.0d, 0.0d)));
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

  void setOrientationContraints(moveit::planning_interface::MoveGroupInterface &group, double factor_pi = 0.3)
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
    ocm.absolute_x_axis_tolerance = factor_pi * M_PI;
    ocm.absolute_y_axis_tolerance = factor_pi * M_PI;
    ocm.absolute_z_axis_tolerance = 2.0 * M_PI;
    ocm.weight = 0.8;

    constr.orientation_constraints.push_back(ocm);

    group.setPathConstraints(constr);
  }

  int updatePlanningScene(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
                          ros::NodeHandle &nh, moveit::planning_interface::MoveGroupInterface &group)
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
      bool found_roof = false;
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
    for (auto &&object : attached_objects)
    {
      group.detachObject(object.first);
    }
    return 1;
  }

  moveit::planning_interface::MoveItErrorCode move(moveit::planning_interface::MoveGroupInterface &group, double dx = 0.0,
                                                   double dy = 0.0, double dz = 0.0, double droll = 0.0,
                                                   double dpitch = 0.0, double dyaw = 0.0)
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
    bool success = (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO("Move planning (pose goal) %s", success ? "" : "FAILED");
    if (success)
    {
      error_code = group.execute(my_plan);
    }
    return error_code;
  }

  moveit::planning_interface::MoveItErrorCode moveToCartPose(moveit::planning_interface::MoveGroupInterface &group,
                                                             Eigen::Isometry3d cartesian_pose,
                                                             std::string base_frame = "mobipick/ur5_base_link",
                                                             std::string target_frame = "mobipick/gripper_tcp")
  {
    robot_state::RobotState start_state(*group.getCurrentState());
    group.setStartState(start_state);

    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = base_frame;
    tf::poseEigenToMsg(cartesian_pose, target_pose.pose);

    group.setPoseTarget(target_pose, target_frame);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    auto error_code = group.plan(my_plan);
    bool success = (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    ROS_INFO("Move planning (pose goal) %s", success ? "" : "FAILED");
    if (success)
    {
      error_code = group.execute(my_plan);
    }
    return error_code;
  }

  bool pause_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    if (!paused)
    {
      ROS_INFO_STREAM("Pause behavior tree after current task is completed");
      paused = true;
    }
    return true;
  }

  bool continue_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    if (paused || failed)
    {
      paused = false;
      failed = false;
      ROS_INFO_STREAM("Continue behavior tree");
    }
    return true;
  }

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> group_ptr;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface::Plan> plan_ptr;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_ptr;

  std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> move_base_ac_ptr;
  std::unique_ptr<actionlib::SimpleActionClient<mobipick_pick_n_place::FtObserverAction>> ft_observer_ac_ptr;
  std::unique_ptr<actionlib::SimpleActionClient<control_msgs::GripperCommandAction>> gripper_ac_ptr;

  /* This is a custom behavior tree implementation which can be paused after any task.
   * Tree execution is exited on pause and can be resumed by running the behavior tree again.
   * A task's behavior is intended to be implemented in its main() method.
   * By default, each node's preprocess() method outputs the name of the current task.
   * When using the default get_next_task() implementation to retrieve the next task's name,
   * an inner tree node returns itself if it has not been run at all. Otherwise,
   * it returns its first unfinished child node. This implementation aims at more meaningful
   * task descriptions by assuming inner nodes to describe coherent sub-behaviors.
   * Details of such a sub-behavior is revealed when it is visited and run.
   */
  class BehaviorTree
  {
  public:
    class Node
    {
    protected:
      bool visited = false;

    public:
      virtual std::string get_task_name() = 0;
      virtual void preprocess() {}
      virtual bool main() = 0;
      virtual void postprocess() {}
      virtual Node *get_next_task() = 0;
      bool run()
      {
        visited = true;
        preprocess();
        bool result = main();
        postprocess();
        return result;
      }
    };

    class TaskNode : public Node
    {
    public:
      virtual void preprocess()
      {
        ROS_INFO_STREAM("TASK " << get_task_name());
      }
      virtual Node *get_next_task()
      {
        return this;
      }
    };

    class InnerNode : public Node
    {
    protected:
      std::string base_name;
      std::vector<Node *> children;
      int child_index = 0;

    public:
      InnerNode(const std::string &name, std::initializer_list<Node *> &&new_children) : base_name(name)
      {
        add_children(new_children);
      }
      virtual Node *get_next_task()
      {
        // If this inner node has not been run yet, return itself as a whole task.
        if (!visited)
        {
          return this;
        }

        // Otherwise return the first incomplete task of the children.
        for (int index = child_index; index < children.size(); ++index)
        {
          Node *next_task = children[index]->get_next_task();
          if (next_task != nullptr)
          {
            return next_task;
          }
        }

        return nullptr;
      }
      void add_child(Node *child) { children.push_back(child); }
      void add_children(std::initializer_list<Node *> &&new_children)
      {
        for (Node *child : new_children)
        {
          add_child(child);
        }
      }
      template <typename CONTAINER>
      void add_children(const CONTAINER &new_children)
      {
        for (Node *child : new_children)
        {
          add_child(child);
        }
      }
    };

    class Selector : public InnerNode
    {
    public:
      using InnerNode::InnerNode;
      virtual std::string get_task_name() { return "Selector " + base_name; }
      virtual bool main() override
      {
        // If this node has been completed before, return true.
        if (child_index < 0)
        {
          return true;
        }

        for (int index = child_index; index < children.size(); ++index)
        {
          // If a child returns true, remember this node as completed.
          if (children[index]->run())
          {
            child_index = -1;
            return true;
          }

          // Note: Do not increase child_index yet, if paused. Both failure and pause of a child return false,
          //   so child_index needs to be revisited on continuation in case it was not completed due to a pause.
          //   Ideally, every node would report both a result and a completion flag. This has been omitted
          //   in this implementation for simplicity.
          if (paused)
          {
            return false;
          }

          child_index = index + 1;
        }
        return false;
      }
    };

    class Sequence : public InnerNode
    {
    public:
      using InnerNode::InnerNode;
      virtual std::string get_task_name() { return "Sequence " + base_name; }
      virtual bool main() override
      {
        for (int index = child_index; index < children.size(); ++index)
        {
          if (!children[index]->run())
          {
            return false;
          }

          child_index = index + 1;
          if (paused)
          {
            return false;
          }
        }

        return true;
      }
    };

  private:
    Node *root;

  public:
    BehaviorTree(Node *main_node) : root(main_node) {}
    std::string get_next_task_name()
    {
      Node *next_task = root->get_next_task();
      return (next_task != nullptr ? next_task->get_task_name() : "No more tasks in behavior tree!");
    }
    bool run() const { return root->run(); }
  };

  class InitTask : public BehaviorTree::TaskNode
  {
  public:
    virtual std::string get_task_name() override { return "Init"; }
    virtual bool main() override
    {
      group_ptr->setPlanningTime(45.0);
      group_ptr->setPlannerId("RRTConnect");
      // wait for the move base action server to come up
      while (!move_base_ac_ptr->waitForServer(ros::Duration(5.0)))
      {
        ROS_INFO("Waiting for the move_base action server to come up");
      }
      ROS_INFO("Connected to mb action server");

      // wait for the ft observer action server to come up
      while (!ft_observer_ac_ptr->waitForServer(ros::Duration(5.0)))
      {
        ROS_INFO("Waiting for the ft_observer action server to come up");
      }
      ROS_INFO("Connected to ft observer action server");

      // wait for the gripper action server to come up
      while (!gripper_ac_ptr->waitForServer(ros::Duration(5.0)))
      {
        ROS_INFO("Waiting for the gripper action server to come up");
      }
      ROS_INFO("Connected to gripper action server");
      return true;
    }
  };

  bool hasAttachedObjects()
  {
    return !planning_scene_interface_ptr->getAttachedObjects().empty();
  }

  class HasAttachedObjectsTask : public BehaviorTree::TaskNode
  {
  private:
    const bool check;

  public:
    HasAttachedObjectsTask(const bool &check) : check(check) {}
    virtual std::string get_task_name() override
    {
      return "HasAttachedObjects(" + std::string(check ? "true" : "false") + ")"; // Note: std::string is required for +.
    }
    virtual bool main() override
    {
      return hasAttachedObjects() == check;
    }
  };

  class MoveArmForDriveTask : public BehaviorTree::TaskNode
  {
  public:
    virtual std::string get_task_name() override { return "MoveArmForDrive"; }
    virtual bool main() override
    {
      const std::string target_name = (hasAttachedObjects() ? "transport" : "home");
      const std::string TARGET_NAME = boost::to_upper_copy<std::string>(target_name);

      /* ******************* MOVE ARM FOR DRIVE ****************************** */
      // plan
      group_ptr->setPlannerId("RRTConnect");
      group_ptr->setStartStateToCurrentState();
      group_ptr->setNamedTarget(target_name);
      moveit::planning_interface::MoveItErrorCode error_code = group_ptr->plan(*plan_ptr);
      if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
        ROS_INFO_STREAM("Planning to " << target_name << " pose SUCCESSFUL");
      }
      else
      {
        ROS_ERROR_STREAM("Planning to " << target_name << " pose FAILED");
        failed = true;
      }

      // move
      error_code = group_ptr->execute(*plan_ptr);
      if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
        ROS_INFO_STREAM("Moving to " << target_name << " pose SUCCESSFUL");
      }
      else
      {
        ROS_ERROR_STREAM("Moving to " << target_name << " pose FAILED");
        failed = true;
      }
      return !failed;
    }
  };

  class MoveBaseTask : public BehaviorTree::TaskNode
  {
  private:
    const std::string target_name;
    const std::string TARGET_NAME;
    const geometry_msgs::Pose goal_pose;

  public:
    MoveBaseTask(const std::string &target_name, const geometry_msgs::Pose &pose) : target_name(target_name),
                                                                                    TARGET_NAME(boost::to_upper_copy<std::string>(target_name)), goal_pose(pose) {}
    virtual std::string get_task_name() override { return "MoveBase(" + target_name + ")"; }
    virtual bool main() override
    {
      /* ******************* MOVE TO TABLE ****************************** */

      move_base_msgs::MoveBaseGoal mb_goal;
      mb_goal.target_pose.header.frame_id = "map";
      mb_goal.target_pose.header.stamp = ros::Time::now();

      mb_goal.target_pose.pose = goal_pose;
      ROS_INFO_STREAM("Send base to " << target_name << " pose and wait...");
      move_base_ac_ptr->sendGoal(mb_goal);

      move_base_ac_ptr->waitForResult();

      if (move_base_ac_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO_STREAM("Moving base to " << target_name << " pose SUCCESSFUL");
      }
      else
      {
        ROS_INFO_STREAM("Moving base to " << target_name << " pose FAILED");
        failed = true;
      }
      return !failed;
    }
  };

  class DriveToNode : public BehaviorTree::Sequence
  {
  private:
    static MoveArmForDriveTask move_arm_for_drive_task;

  public:
    DriveToNode(const std::string &target_name, const geometry_msgs::Pose &pose)
        : BehaviorTree::Sequence("DriveTo(" + target_name + ")", {&move_arm_for_drive_task,
                                                                  new MoveBaseTask(target_name, pose)}) {}
  };

  MoveArmForDriveTask DriveToNode::move_arm_for_drive_task;

  class CaptureObjectTask : public BehaviorTree::TaskNode
  {
  public:
    virtual std::string get_task_name() override { return "CaptureObject"; }
    virtual bool main() override
    {
      /* ********************* PLAN AND EXECUTE MOVES ********************* */

      // plan to observe the table
      group_ptr->setNamedTarget("observe100cm_right");
      moveit::planning_interface::MoveItErrorCode error_code = group_ptr->plan(*plan_ptr);
      if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
        ROS_INFO("Planning to observation pose SUCCESSFUL");
      }
      else
      {
        ROS_ERROR("Planning to observation pose FAILED");
        failed = true;
      }

      // move to observation pose
      error_code = group_ptr->execute(*plan_ptr);
      if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
        ROS_INFO("Moving to observation pose SUCCESSFUL");
      }
      else
      {
        ROS_ERROR("Moving to observation pose FAILED");
        failed = true;
      }
      ros::WallDuration(5.0).sleep();
      return !failed;
    }
  };

  class PickUpObjectTask : public BehaviorTree::TaskNode
  {
  private:
    ros::NodeHandle nh;

  public:
    PickUpObjectTask(ros::NodeHandle &nh) : nh(nh) {}
    virtual std::string get_task_name() override { return "PickUpObject"; }
    virtual bool main() override
    {
      /* ********************* PICK ********************* */

      // clear octomap
      ros::ServiceClient clear_octomap = nh.serviceClient<std_srvs::Empty>("clear_octomap");
      std_srvs::Empty srv;
      group_ptr->clearPathConstraints();
      // pick
      moveit::planning_interface::MoveItErrorCode error_code;
      uint pickPlanAttempts = 0;
      do
      {
        clear_octomap.call(srv);

        updatePlanningScene(*planning_scene_interface_ptr, nh, *group_ptr);
        error_code = pick(*group_ptr);
        ++pickPlanAttempts;

        if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
          ROS_INFO("Picking SUCCESSFUL");
        }
        else if ((error_code == moveit::planning_interface::MoveItErrorCode::PLANNING_FAILED ||
                  error_code == moveit::planning_interface::MoveItErrorCode::INVALID_MOTION_PLAN) &&
                 pickPlanAttempts < 10)
        {
          ROS_INFO("Planning for Picking FAILED");
        }
        else
        {
          ROS_ERROR("Picking FAILED");
          failed = true;
        }

      } while ((error_code == moveit::planning_interface::MoveItErrorCode::PLANNING_FAILED ||
                error_code == moveit::planning_interface::MoveItErrorCode::INVALID_MOTION_PLAN) &&
               pickPlanAttempts < 10);
      return !failed;
    }
  };

  class MoveArmToHandoverTask : public BehaviorTree::TaskNode
  {
  public:
    virtual std::string get_task_name() override { return "MoveArmToHandover"; }
    virtual bool main() override
    {
      /* ********************* PLAN AND EXECUTE TO HAND OVER POSE ********************* */
      setOrientationContraints(*group_ptr, 0.666666);
      Eigen::Isometry3d hand_over_pose = Eigen::Isometry3d::Identity();
      hand_over_pose.translate(Eigen::Vector3d(0.8, -0.4, 0.2));
      hand_over_pose.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0.0, 1.0, 0.0)));
      hand_over_pose.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(1.0, 0.0, 0.0)));
      hand_over_pose.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0.0, 0.0, 1.0)));

      if (moveToCartPose(*group_ptr, hand_over_pose) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
        ROS_INFO("Move arm to HANDOVER succeeded");
      }
      else
      {
        ROS_INFO("Move arm to HANDOVER failed");
        failed = true;
      }
      return !failed;
    }
  };

  class UserInteractionTask : public BehaviorTree::TaskNode
  {
  public:
    virtual std::string get_task_name() override { return "UserInteraction"; }
    virtual bool main() override
    {
      /* ********************* WAIT FOR USER ********************* */
      mobipick_pick_n_place::FtObserverGoal ft_goal;

      ft_goal.threshold = 5.0;
      ft_goal.timeout = 30.0;

      ft_observer_ac_ptr->sendGoal(ft_goal);

      ft_observer_ac_ptr->waitForResult();

      if (ft_observer_ac_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("Detection user interaction SUCCESSFUL");
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
          for (auto &&object : attached_objects)
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
          failed = true;
        }
      }
      else
      {
        // Note: Task still counts as success, so the behavior tree continues. However, trigger a pause.
        ROS_INFO("Detection user interaction FAILED, start placing Object");
        paused = true;
        failed = false; // TODO
      }
      return !failed;
    }
  };

  class PlaceObjectTask : public BehaviorTree::TaskNode
  {
  public:
    virtual std::string get_task_name() override { return "PlaceObject"; }
    virtual bool main() override
    {
      /* ********************* PLACE ********************* */

      // move(group, -0.05, -0.05, 0.2);
      ros::WallDuration(1.0).sleep();
      group_ptr->setPlannerId("RRTConnect");
      ROS_INFO("Start Placing");
      // place
      uint placePlanAttempts = 0;
      moveit::planning_interface::MoveItErrorCode error_code;
      do
      {
        group_ptr->setPlanningTime(40 + 10 * placePlanAttempts);
        setOrientationContraints(*group_ptr, 0.3);
        error_code = place(*group_ptr);
        ++placePlanAttempts;
        if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
          ROS_INFO("Placing SUCCESSFUL");
        }
        else if ((error_code == moveit::planning_interface::MoveItErrorCode::PLANNING_FAILED ||
                  error_code == moveit::planning_interface::MoveItErrorCode::INVALID_MOTION_PLAN ||
                  error_code == moveit::planning_interface::MoveItErrorCode::TIMED_OUT) &&
                 placePlanAttempts < 10)
        {
          ROS_INFO("Planning for Placing FAILED");
          ros::WallDuration(1.0).sleep();
          // move(group, 0.01, 0.01, -0.01); //TODO: make a random/suitable move
        }
        else
        {
          ROS_ERROR("Placing FAILED");
          failed = true;
        }
      } while ((error_code == moveit::planning_interface::MoveItErrorCode::PLANNING_FAILED ||
                error_code == moveit::planning_interface::MoveItErrorCode::INVALID_MOTION_PLAN ||
                error_code == moveit::planning_interface::MoveItErrorCode::TIMED_OUT) &&
               placePlanAttempts < 10);
      return !failed;
    }
  };

  class DoneTask : public BehaviorTree::TaskNode
  {
  public:
    virtual std::string get_task_name() override { return "Done"; }
    virtual bool main() override
    {
      return true;
    }
  };

}

using namespace mobipick;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mobipick_pick_n_place");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;

  geometry_msgs::Pose base_pick_pose;
  geometry_msgs::Pose base_handover_pose;
  geometry_msgs::Pose base_place_pose;
  geometry_msgs::Pose base_home_pose;
  std::string world_name;

  // Load rosparams
  ros::NodeHandle rpnh(nh, "poses");
  std::size_t error = 0;
  error += !rosparam_shortcuts::get("poses", rpnh, "base_pick_pose", base_pick_pose);         // geometry_msgs::Pose base_pick_pose
  error += !rosparam_shortcuts::get("poses", rpnh, "base_handover_pose", base_handover_pose); // geometry_msgs::Pose base_handover_pose
  error += !rosparam_shortcuts::get("poses", rpnh, "base_place_pose", base_place_pose);       // geometry_msgs::Pose base_place_pose
  error += !rosparam_shortcuts::get("poses", rpnh, "base_home_pose", base_home_pose);         // geometry_msgs::Pose base_home_pose
  error += !rosparam_shortcuts::get("poses", rpnh, "world_name", world_name);                 // string
  // add more parameters here to load if desired
  rosparam_shortcuts::shutdownIfError("poses", error);

  ROS_INFO_STREAM("Current world name: " << world_name);

  // The Get Tool Task lets mobipick fetch the tool from the table if it is not holding any objects already.
  BehaviorTree::Selector get_tool_task("GetTool", {new HasAttachedObjectsTask(true),
                                                   new BehaviorTree::Sequence("FetchTool", {new DriveToNode("pick", base_pick_pose),
                                                                                            new CaptureObjectTask(),
                                                                                            new PickUpObjectTask(nh)})});

  // The Clear Gripper Task lets mobipick place any object it is holding onto the table.
  BehaviorTree::Selector clear_gripper_task("ClearGripper", {new HasAttachedObjectsTask(false),
                                                             new BehaviorTree::Sequence("StashTool", {new DriveToNode("place", base_place_pose),
                                                                                                      new PlaceObjectTask()})});

  // The PickNPlace Behavior Tree lets mobipick fetch the tool, offer it to a person, stow it away if needed,
  //   and finally drive back home.
  BehaviorTree behavior_tree(new BehaviorTree::Sequence("PickNPlace", {new InitTask(),
                                                                       &get_tool_task,
                                                                       new DriveToNode("handover", base_handover_pose),
                                                                       new MoveArmToHandoverTask(),
                                                                       new UserInteractionTask(),
                                                                       &clear_gripper_task,
                                                                       new DriveToNode("home", base_home_pose),
                                                                       new DoneTask()}));

  // pause service
  ros::ServiceServer pause_state = nh.advertiseService("pause_behavior_tree", pause_service);

  // pause service
  ros::ServiceServer continue_state = nh.advertiseService("continue_behavior_tree", continue_service);

  group_ptr = std::make_unique<moveit::planning_interface::MoveGroupInterface>("arm");
  // MOVE BASE
  move_base_ac_ptr = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
  // FT Observer
  ft_observer_ac_ptr = std::make_unique<actionlib::SimpleActionClient<mobipick_pick_n_place::FtObserverAction>>("ft_observer", true);
  // GRIPPER
  gripper_ac_ptr = std::make_unique<actionlib::SimpleActionClient<control_msgs::GripperCommandAction>>("gripper_hw", true);
  // MOVE IT
  plan_ptr = std::make_unique<moveit::planning_interface::MoveGroupInterface::Plan>();
  // PLANNING INTERFACE
  planning_scene_interface_ptr = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();

  while (ros::ok())
  {
    if (paused || failed)
    {
      if (!pause_active)
      {
        ROS_INFO_STREAM("PAUSED before task: " << behavior_tree.get_next_task_name());
        ROS_INFO_STREAM("Call service continue_behavior_tree to resume");
        pause_active = true;
      }
      continue;
    }

    if (pause_active)
    {
      ROS_INFO_STREAM("Next task: " << behavior_tree.get_next_task_name());
      pause_active = false;
    }
    if (behavior_tree.run())
    {
      ROS_INFO("Behavior tree completed.");
      return 0;
    }
  }
  return 1;
}
