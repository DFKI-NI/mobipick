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

#include "mobipick_pick_n_place/pausable_behavior_tree.h"

#include <ros/ros.h>

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
bool paused = true;
bool failed = false;
bool pause_active = false;

bool pause_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  if (!paused)
  {
    ROS_INFO_STREAM("Pause behavior tree after current task is completed");
    paused = true;
  }
  return true;
}

bool continue_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  if (paused || failed)
  {
    paused = false;
    failed = false;
    ROS_INFO_STREAM("Continue behavior tree");
  }
  return true;
}

std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> move_base_ac_ptr;
std::unique_ptr<actionlib::SimpleActionClient<mobipick_pick_n_place::FtObserverAction>> ft_observer_ac_ptr;
std::unique_ptr<actionlib::SimpleActionClient<mobipick_pick_n_place::MoveItMacroAction>> moveit_macros_ac_ptr;

class ROSTaskNode : public BehaviorTree::TaskNode
{
public:
  virtual void preprocess()
  {
    ROS_INFO_STREAM("TASK " << get_task_name());
  }
};

class InitTask : public ROSTaskNode
{
public:
  virtual std::string get_task_name() override
  {
    return "Init";
  }
  virtual bool main() override
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
    return true;
  }
};

bool hasAttachedObjects()
{
  mobipick_pick_n_place::MoveItMacroGoal goal;
  goal.type = "function";
  goal.name = "HasAttachedObjects";
  moveit_macros_ac_ptr->sendGoal(goal);
  moveit_macros_ac_ptr->waitForResult();
  return moveit_macros_ac_ptr->getResult()->result;
}

class HasAttachedObjectsTask : public ROSTaskNode
{
private:
  const bool check;

public:
  HasAttachedObjectsTask(const bool& check) : check(check)
  {
  }
  virtual std::string get_task_name() override
  {
    return "HasAttachedObjects(check if " + std::string(check ? "true" : "false") +
           ")";  // Note: std::string is required for +.
  }
  virtual bool main() override
  {
    return hasAttachedObjects() == check;
  }
};

class MoveArmForDriveTask : public ROSTaskNode
{
public:
  virtual std::string get_task_name() override
  {
    return "MoveArmForDrive";
  }
  virtual bool main() override
  {
    const std::string target_name = (hasAttachedObjects() ? "transport" : "home");
    const std::string TARGET_NAME = boost::to_upper_copy<std::string>(target_name);

    mobipick_pick_n_place::MoveItMacroGoal goal;
    goal.type = "target";
    goal.name = target_name;
    moveit_macros_ac_ptr->sendGoal(goal);
    moveit_macros_ac_ptr->waitForResult();
    failed = !moveit_macros_ac_ptr->getResult()->result;
    return !failed;
  }
};

class MoveBaseTask : public ROSTaskNode
{
private:
  const std::string target_name;
  const std::string TARGET_NAME;
  const geometry_msgs::Pose goal_pose;

public:
  MoveBaseTask(const std::string& target_name, const geometry_msgs::Pose& pose)
    : target_name(target_name), TARGET_NAME(boost::to_upper_copy<std::string>(target_name)), goal_pose(pose)
  {
  }
  virtual std::string get_task_name() override
  {
    return "MoveBase(" + target_name + ")";
  }
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
  DriveToNode(const std::string& target_name, const geometry_msgs::Pose& pose)
    : BehaviorTree::Sequence("DriveTo(" + target_name + ")",
                             { &move_arm_for_drive_task, new MoveBaseTask(target_name, pose) })
  {
  }
};

MoveArmForDriveTask DriveToNode::move_arm_for_drive_task;

class MoveItTask : public ROSTaskNode
{
private:
  const std::string name;

public:
  MoveItTask(const std::string& name) : name(name)
  {
  }
  virtual std::string get_task_name() override
  {
    return name;
  }
  virtual bool main() override
  {
    mobipick_pick_n_place::MoveItMacroGoal goal;
    goal.type = "function";
    goal.name = name;
    moveit_macros_ac_ptr->sendGoal(goal);
    moveit_macros_ac_ptr->waitForResult();
    failed = !moveit_macros_ac_ptr->getResult()->result;
    return !failed;
  }
};

class UserInteractionTask : public ROSTaskNode
{
public:
  virtual std::string get_task_name() override
  {
    return "UserInteraction";
  }
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
      mobipick_pick_n_place::MoveItMacroGoal goal;
      goal.type = "function";
      goal.name = "ReleaseGripper";
      moveit_macros_ac_ptr->sendGoal(goal);
      moveit_macros_ac_ptr->waitForResult();
      failed = !moveit_macros_ac_ptr->getResult()->result;
    }
    else
    {
      // Note: Task still counts as success, so the behavior tree continues. However, trigger a pause.
      ROS_INFO("Detection user interaction FAILED, start placing Object");
      paused = true;
      failed = false;
    }
    return !failed;
  }
};

class DoneTask : public ROSTaskNode
{
public:
  virtual std::string get_task_name() override
  {
    return "Done";
  }
  virtual bool main() override
  {
    return true;
  }
};

}  // namespace mobipick

using namespace mobipick;

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
  ros::NodeHandle rpnh(nh, "poses");
  std::size_t error = 0;
  error +=
      !rosparam_shortcuts::get("poses", rpnh, "base_pick_pose", base_pick_pose);  // geometry_msgs::Pose base_pick_pose
  error += !rosparam_shortcuts::get("poses", rpnh, "base_handover_pose",
                                    base_handover_pose);  // geometry_msgs::Pose base_handover_pose
  error += !rosparam_shortcuts::get("poses", rpnh, "base_place_pose",
                                    base_place_pose);  // geometry_msgs::Pose base_place_pose
  error +=
      !rosparam_shortcuts::get("poses", rpnh, "base_home_pose", base_home_pose);  // geometry_msgs::Pose base_home_pose
  error += !rosparam_shortcuts::get("poses", rpnh, "world_name", world_name);     // string
  // add more parameters here to load if desired
  rosparam_shortcuts::shutdownIfError("poses", error);

  ROS_INFO_STREAM("Current world name: " << world_name);

  // The Get Tool Task lets mobipick fetch the tool from the table if it is not holding any objects already.
  BehaviorTree::Selector get_tool_task(
      "GetTool",
      { new HasAttachedObjectsTask(true),
        new BehaviorTree::Sequence("FetchTool", { new DriveToNode("pick", base_pick_pose),
                                                  new MoveItTask("CaptureObject"), new MoveItTask("PickUpObject") }) });

  // The Clear Gripper Task lets mobipick place any object it is holding onto the table.
  BehaviorTree::Selector clear_gripper_task(
      "ClearGripper", { new HasAttachedObjectsTask(false),
                        new BehaviorTree::Sequence("StashTool", { new DriveToNode("place", base_place_pose),
                                                                  new MoveItTask("PlaceObject") }) });

  // The PickNPlace Behavior Tree lets mobipick fetch the tool, offer it to a person, stow it away if needed,
  //   and finally drive back home.
  BehaviorTree behavior_tree(
      new BehaviorTree::Sequence("PickNPlace",
                                 { new InitTask(), &get_tool_task, new DriveToNode("handover", base_handover_pose),
                                   new MoveItTask("MoveArmToHandover"), new UserInteractionTask(), &clear_gripper_task,
                                   new DriveToNode("home", base_home_pose), new DoneTask() }),
      &paused);

  // pause service
  ros::ServiceServer pause_state = nh.advertiseService("pause_behavior_tree", pause_service);

  // pause service
  ros::ServiceServer continue_state = nh.advertiseService("continue_behavior_tree", continue_service);

  // MOVE BASE
  move_base_ac_ptr = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>("move_base", true);
  // FT Observer
  ft_observer_ac_ptr =
      std::make_unique<actionlib::SimpleActionClient<mobipick_pick_n_place::FtObserverAction>>("ft_observer", true);
  // MOVE IT MACROS
  moveit_macros_ac_ptr =
      std::make_unique<actionlib::SimpleActionClient<mobipick_pick_n_place::MoveItMacroAction>>("moveit_macros", true);

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
