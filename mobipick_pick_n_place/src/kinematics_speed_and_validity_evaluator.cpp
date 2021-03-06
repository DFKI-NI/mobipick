/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
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

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/profiler/profiler.h>
#include <ros/ros.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";

// from https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
double angle_diff(double x, double y)
{
  return atan2(sin(x - y), cos(x - y));
}

double angle_diffs(std::vector<double> xs, std::vector<double> ys)
{
  double result = 0.0;
  assert(xs.size() == ys.size());
  for (size_t i = 0; i < xs.size(); ++i)
  {
    result += fabs(angle_diff(xs[i], ys[i])) / (double)xs.size();
  }
  return result;
}

double unnormalized_angle_diffs(std::vector<double> xs, std::vector<double> ys)
{
  double result = 0.0;
  assert(xs.size() == ys.size());
  for (size_t i = 0; i < xs.size(); ++i)
  {
    result += fabs(xs[i] - ys[i]) / (double)xs.size();
  }
  return result;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "inverse_kinematics_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  if (argc <= 1)
    ROS_ERROR("An argument specifying the group name is needed");
  else
  {
    robot_model_loader::RobotModelLoader rml(ROBOT_DESCRIPTION);
    std::string group = argv[1];
    ROS_INFO_STREAM("Evaluating IK for " << group);

    const robot_model::JointModelGroup* jmg = rml.getModel()->getJointModelGroup(group);
    if (jmg)
    {
      const kinematics::KinematicsBaseConstPtr& solver = jmg->getSolverInstance();
      if (solver)
      {
        const std::string& tip = solver->getTipFrame();
        robot_state::RobotState state(rml.getModel());
        state.setToDefaultValues();

        ROS_INFO_STREAM("Tip Frame:  " << solver->getTipFrame());
        ROS_INFO_STREAM("Base Frame: " << solver->getBaseFrame());
        ROS_INFO_STREAM("IK Timeout: " << solver->getDefaultTimeout());
        ROS_INFO_STREAM("Search res: " << solver->getSearchDiscretization());

        unsigned int test_count = 1000;
        if (argc > 2)
          try
          {
            test_count = boost::lexical_cast<unsigned int>(argv[2]);
          }
          catch (...)
          {
          }

        ROS_INFO("Running %u tests", test_count);

        // specify seed 0 for deterministic behavior
        random_numbers::RandomNumberGenerator rng(0);

        moveit::tools::Profiler::Start();
        for (unsigned int i = 0; i < test_count; ++i)
        {
          state.setToRandomPositions(jmg, rng);
          Eigen::Isometry3d pose = state.getGlobalLinkTransform(tip);

          state.setToRandomPositions(jmg, rng);
          std::vector<double> joint_positions_seed;
          joint_positions_seed.push_back(state.getVariablePosition("ur5_shoulder_pan_joint"));
          joint_positions_seed.push_back(state.getVariablePosition("ur5_shoulder_lift_joint"));
          joint_positions_seed.push_back(state.getVariablePosition("ur5_elbow_joint"));
          joint_positions_seed.push_back(state.getVariablePosition("ur5_wrist_1_joint"));
          joint_positions_seed.push_back(state.getVariablePosition("ur5_wrist_2_joint"));
          joint_positions_seed.push_back(state.getVariablePosition("ur5_wrist_3_joint"));

          moveit::tools::Profiler::Begin("IK");
          state.setFromIK(jmg, pose);
          moveit::tools::Profiler::End("IK");
          const Eigen::Isometry3d& pose_upd = state.getGlobalLinkTransform(tip);
          Eigen::Isometry3d diff = pose_upd * pose.inverse();
          double rot_err = (diff.rotation() - Eigen::Matrix3d::Identity()).norm();
          double trans_err = diff.translation().norm();

          std::vector<double> joint_positions_ik;
          joint_positions_ik.push_back(state.getVariablePosition("ur5_shoulder_pan_joint"));
          joint_positions_ik.push_back(state.getVariablePosition("ur5_shoulder_lift_joint"));
          joint_positions_ik.push_back(state.getVariablePosition("ur5_elbow_joint"));
          joint_positions_ik.push_back(state.getVariablePosition("ur5_wrist_1_joint"));
          joint_positions_ik.push_back(state.getVariablePosition("ur5_wrist_2_joint"));
          joint_positions_ik.push_back(state.getVariablePosition("ur5_wrist_3_joint"));

          moveit::tools::Profiler::Average("Rotation error", rot_err);
          moveit::tools::Profiler::Average("Translation error", trans_err);
          if (rot_err < 1e-3 && trans_err < 1e-3)
          {
            moveit::tools::Profiler::Event("Valid IK");
            moveit::tools::Profiler::Average("Success Rate", 100);
            moveit::tools::Profiler::Average("Normalized   angle distance IK seed - IK result",
                                             angle_diffs(joint_positions_seed, joint_positions_ik));
            moveit::tools::Profiler::Average("Unnormalized angle distance IK seed - IK result",
                                             unnormalized_angle_diffs(joint_positions_seed, joint_positions_ik));
          }
          else
          {
            moveit::tools::Profiler::Event("Invalid IK");
            moveit::tools::Profiler::Average("Success Rate", 0);
          }
        }
        moveit::tools::Profiler::Stop();
        moveit::tools::Profiler::Status();
      }
      else
        ROS_ERROR_STREAM("No kinematics solver specified for group " << group);
    }
  }

  ros::shutdown();
  return 0;
}
