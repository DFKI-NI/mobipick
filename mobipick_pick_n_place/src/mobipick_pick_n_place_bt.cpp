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

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include <ros/ros.h>
#include <XmlRpcValue.h>

// Boost
#include <boost/algorithm/string.hpp>

// Move base
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

// ft observer
#include <mobipick_pick_n_place/FtObserverAction.h>

// MoveIt macros
#include <mobipick_pick_n_place/MoveItMacroAction.h>

#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <vision_msgs/Detection3DArray.h>

#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace mobipick
{
bool failed = false;

std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> move_base_ac_ptr;
std::unique_ptr<actionlib::SimpleActionClient<mobipick_pick_n_place::FtObserverAction>> ft_observer_ac_ptr;
std::unique_ptr<actionlib::SimpleActionClient<mobipick_pick_n_place::MoveItMacroAction>> moveit_macros_ac_ptr;

class InitNode : public BT::SyncActionNode
{
public:
  InitNode(const std::string& name) : BT::SyncActionNode(name, {})
  {
  }
  BT::NodeStatus tick() override
  {
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

    // wait for the moveit macros action server to come up
    while (!moveit_macros_ac_ptr->waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for moveit_macros action server to come up");
    }
    ROS_INFO("Connected to moveit macros action server");
    return BT::NodeStatus::SUCCESS;
  }
};

BT::NodeStatus hasAttachedObjects()
{
  mobipick_pick_n_place::MoveItMacroGoal goal;
  goal.type = "function";
  goal.name = "HasAttachedObjects";
  moveit_macros_ac_ptr->sendGoal(goal);
  moveit_macros_ac_ptr->waitForResult();
  return moveit_macros_ac_ptr->getResult()->result ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

class DriveToNode : public BT::SyncActionNode
{
private:
  const std::map<std::string, geometry_msgs::Pose> base_poses;

public:
  DriveToNode(const std::string& name, const BT::NodeConfiguration& config,
              const std::map<std::string, geometry_msgs::Pose>& base_poses)
    : BT::SyncActionNode(name, config), base_poses(base_poses)
  {
  }
  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("goal") };
  }
  BT::NodeStatus tick() override
  {
    mobipick_pick_n_place::MoveItMacroGoal goal;
    goal.type = "target";
    goal.name = (hasAttachedObjects() == BT::NodeStatus::SUCCESS ? "transport" : "home");
    moveit_macros_ac_ptr->sendGoal(goal);
    moveit_macros_ac_ptr->waitForResult();
    failed = !moveit_macros_ac_ptr->getResult()->result;
    if (failed)
    {
      return BT::NodeStatus::FAILURE;
    }

    /* ******************* MOVE TO GOAL ****************************** */

    BT::Optional<std::string> msg = getInput<std::string>("goal");
    if (!msg)
    {
      throw BT::RuntimeError("missing required input [goal]: ", msg.error());
    }

    move_base_msgs::MoveBaseGoal mb_goal;
    mb_goal.target_pose.header.frame_id = "map";
    mb_goal.target_pose.header.stamp = ros::Time::now();

    std::string target_name = msg.value();
    mb_goal.target_pose.pose = base_poses.at(target_name);
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
    return failed ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }
};

class MoveItNode : public BT::SyncActionNode
{
public:
  MoveItNode(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
  {
  }
  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("function") };
  }
  BT::NodeStatus tick() override
  {
    BT::Optional<std::string> msg = getInput<std::string>("function");
    if (!msg)
    {
      throw BT::RuntimeError("missing required input [function]: ", msg.error());
    }

    mobipick_pick_n_place::MoveItMacroGoal goal;
    goal.type = "function";
    goal.name = msg.value();
    moveit_macros_ac_ptr->sendGoal(goal);
    moveit_macros_ac_ptr->waitForResult();
    failed = !moveit_macros_ac_ptr->getResult()->result;
    return failed ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }
};

class UserInteractionNode : public BT::SyncActionNode
{
public:
  UserInteractionNode(const std::string& name) : BT::SyncActionNode(name, {})
  {
  }
  BT::NodeStatus tick() override
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
      mobipick_pick_n_place::MoveItMacroGoal goal;
      goal.type = "function";
      goal.name = "ReleaseGripper";
      moveit_macros_ac_ptr->sendGoal(goal);
      moveit_macros_ac_ptr->waitForResult();
      failed = !moveit_macros_ac_ptr->getResult()->result;
    }
    else
    {
      ROS_INFO("Detection user interaction FAILED, start placing Object");
      failed = false;
    }
    return failed ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }
};

class DoneNode : public BT::SyncActionNode
{
public:
  DoneNode(const std::string& name) : BT::SyncActionNode(name, {})
  {
  }
  BT::NodeStatus tick() override
  {
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace mobipick

using namespace mobipick;

double cast_value(const XmlRpc::XmlRpcValue& value)
{
  // Note: Casting an int XmlRpcValue to double directly, throws an XmlRpc::XmlRpcException.
  return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? int(value) : double(value);
}

bool get_pose(XmlRpc::XmlRpcValue& poses, const std::string& key, geometry_msgs::Pose& pose)
{
  if (!poses.hasMember(key))
  {
    return false;
  }

  const auto& tokens = poses[key];
  const auto& position = tokens[0];
  const auto& orientation = tokens[1];
  pose.position.x = cast_value(position[0]);
  pose.position.y = cast_value(position[1]);
  pose.position.z = cast_value(position[2]);
  pose.orientation.x = cast_value(orientation[0]);
  pose.orientation.y = cast_value(orientation[1]);
  pose.orientation.z = cast_value(orientation[2]);
  pose.orientation.w = cast_value(orientation[3]);
  return true;
}

int main(int argc, char** argv)
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
  XmlRpc::XmlRpcValue poses;
  ros::NodeHandle rpnh(nh, "poses");
  std::size_t error = 0;
  error += !nh.getParam("poses", poses);
  std::string base_pick_pose_name = nh.param<std::string>("base_pick_pose_name", "base_pick_pose");
  error += !get_pose(poses, base_pick_pose_name, base_pick_pose);       // geometry_msgs::Pose base_pick_pose
  error += !get_pose(poses, "base_handover_pose", base_handover_pose);  // geometry_msgs::Pose base_handover_pose
  std::string base_place_pose_name = nh.param<std::string>("base_place_pose_name", "base_place_pose");
  error += !get_pose(poses, base_place_pose_name, base_place_pose);            // geometry_msgs::Pose base_place_pose
  error += !get_pose(poses, "base_home_pose", base_home_pose);                 // geometry_msgs::Pose base_home_pose
  error += !rosparam_shortcuts::get("poses", rpnh, "world_name", world_name);  // string
  // add more parameters here to load if desired
  rosparam_shortcuts::shutdownIfError("poses", error);

  std::map<std::string, geometry_msgs::Pose> base_poses{ { "pick", base_pick_pose },
                                                         { "handover", base_handover_pose },
                                                         { "place", base_place_pose },
                                                         { "home", base_home_pose } };

  ROS_INFO_STREAM("Current world name: " << world_name);

  // Get behavior tree config filepath from command line arguments.
  char* behavior_tree_filepath = nullptr;
  for (char** pargv = argv + 1; *pargv != argv[argc]; ++pargv)
  {
    if (boost::ends_with(*pargv, "_bt.xml"))
    {
      behavior_tree_filepath = *pargv;
      break;
    }
  }
  if (behavior_tree_filepath == nullptr)
  {
    ROS_ERROR("Behavior tree filepath *_bt.xml not specified in command line arguments.");
    std::exit(2);
  }

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<DoneNode>("DoneNode");
  factory.registerNodeType<InitNode>("InitNode");
  factory.registerSimpleCondition("HasAttachedObjects", std::bind(hasAttachedObjects));
  BT::NodeBuilder builder_drive_to = [&base_poses](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<DriveToNode>(name, config, base_poses);
  };
  factory.registerBuilder<DriveToNode>("DriveToNode", builder_drive_to);
  factory.registerNodeType<MoveItNode>("MoveItNode");
  factory.registerNodeType<UserInteractionNode>("UserInteractionNode");
  BT::Tree tree = factory.createTreeFromFile(behavior_tree_filepath);
  BT::PublisherZMQ publisher_zmq(tree);

  // MOVE BASE
  move_base_ac_ptr = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
  // FT Observer
  ft_observer_ac_ptr =
      std::make_unique<actionlib::SimpleActionClient<mobipick_pick_n_place::FtObserverAction>>("ft_observer", true);
  // MOVE IT MACROS
  moveit_macros_ac_ptr =
      std::make_unique<actionlib::SimpleActionClient<mobipick_pick_n_place::MoveItMacroAction>>("moveit_macros", true);

  if (ros::ok())
  {
    if (tree.tickRoot() == BT::NodeStatus::SUCCESS)
    {
      ROS_INFO("Behavior tree completed.");
      return 0;
    }
  }
  return 1;
}
