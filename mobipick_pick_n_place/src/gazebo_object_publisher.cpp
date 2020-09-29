#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <vision_msgs/Detection3DArray.h>
#include <geometry_msgs/PoseStamped.h>
#include "mobipick_pick_n_place/fake_object_recognition.h"
#include <eigen_conversions/eigen_msg.h>

std::shared_ptr<ros::Publisher> detection_pub;
std::shared_ptr<ros::Publisher> pose_power_drill_pub;
std::shared_ptr<ros::Publisher> pose_table_pub;


void gazebo_cb(const gazebo_msgs::LinkStatesConstPtr msg)
{
  // input topic has 1000 Hz, output topic should have 10 Hz
  static size_t counter = 0;
  counter = (counter + 1) % 100;
  if (counter != 0)
    return;

  vision_msgs::Detection3DArray detections;

  detections.header.stamp = ros::Time::now();
  detections.header.frame_id = "mobipick/base_link";

  assert(msg->name.size() == msg->pose.size());
  bool base_link_found = false;
  Eigen::Isometry3d mobipick_pose;
  while (!base_link_found)
  {
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      // TODO: Change to tf listener?
      if (msg->name[i] == "mobipick::mobipick/base_footprint" && !base_link_found)
      {
        base_link_found= true;
        tf::poseMsgToEigen(msg->pose[i], mobipick_pose);
        mobipick_pose = mobipick_pose.inverse();
      }
      if (base_link_found)
      {
        if (msg->name[i] == "table_1::table_top_link")
        {
          // add table
          vision_msgs::Detection3D det3d;
          det3d.header = detections.header;
          det3d.bbox.size.x = 0.70;
          det3d.bbox.size.y = 0.70;
          det3d.bbox.size.z = 0.73;

          // shift to center of bbox
          Eigen::Isometry3d object_pose;
          tf::poseMsgToEigen(msg->pose[i], object_pose);
          Eigen::Isometry3d object_to_bbox = Eigen::Isometry3d::Identity();
          object_to_bbox.translation() = Eigen::Vector3d(0.0d, 0.0d, det3d.bbox.size.z / 2.0d);
          tf::poseEigenToMsg((mobipick_pose * object_pose * object_to_bbox), det3d.bbox.center);

          det3d.results.resize(1);
          det3d.results[0].id = ObjectID::TABLE;
          tf::poseEigenToMsg((mobipick_pose * object_pose), det3d.results[0].pose.pose);
          det3d.results[0].score = 1.0;
          detections.detections.push_back(det3d);
          // publish table pose seperately
          geometry_msgs::PoseStamped pose_msg;
          pose_msg.header = det3d.header;
          pose_msg.pose = det3d.bbox.center;
          pose_table_pub->publish(pose_msg);
        }
        if (msg->name[i] == "cokecan_1::coke_can")
        {
          // add coke_can
          vision_msgs::Detection3D det3d;
          det3d.header = detections.header;
          det3d.bbox.size.x = 0.067;
          det3d.bbox.size.y = 0.067;
          det3d.bbox.size.z = 0.1239;

          // shift to center of bbox
          Eigen::Isometry3d object_pose;
          tf::poseMsgToEigen(msg->pose[i], object_pose);
          Eigen::Isometry3d object_to_bbox = Eigen::Isometry3d::Identity();
          object_to_bbox.translation() = Eigen::Vector3d(0.0d, 0.0d, det3d.bbox.size.z / 2.0d);
          tf::poseEigenToMsg((mobipick_pose * object_pose * object_to_bbox), det3d.bbox.center);

          det3d.results.resize(1);
          det3d.results[0].id = ObjectID::COKE_CAN;
          tf::poseEigenToMsg((mobipick_pose * object_pose), det3d.results[0].pose.pose);
          det3d.results[0].score = 1.0;
          detections.detections.push_back(det3d);
        }
        if (msg->name[i] == "powerdrill_1::powerdrill")
        {
          
          // add powerdrill
          vision_msgs::Detection3D det3d;
          det3d.header = detections.header;
          det3d.bbox.size.x = 0.184208;
          det3d.bbox.size.y = 0.187514;
          det3d.bbox.size.z = 0.057294;

          // shift to center of bbox and rotate coordinate frames to match dope detection
          Eigen::Isometry3d object_pose;
          tf::poseMsgToEigen(msg->pose[i], object_pose);

          Eigen::Isometry3d object_to_bbox = Eigen::Isometry3d::Identity();
          object_to_bbox.translation() = Eigen::Vector3d(-0.046d, 0.01055d, 0.02545d);

          // rotate to dope orientation
          Eigen::Isometry3d bbox_center_rotated = Eigen::Isometry3d::Identity();
          bbox_center_rotated.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(1.0d, 0.0d, 0.0d)));
          tf::poseEigenToMsg((mobipick_pose * object_pose * object_to_bbox * bbox_center_rotated), det3d.bbox.center);

          det3d.results.resize(1);
          det3d.results[0].id = ObjectID::POWER_DRILL;
          tf::poseEigenToMsg((mobipick_pose * object_pose * bbox_center_rotated), det3d.results[0].pose.pose);
          det3d.results[0].score = 1.0;
          detections.detections.push_back(det3d);
          // publish power drill pose seperately
          geometry_msgs::PoseStamped pose_msg;
          pose_msg.header = det3d.header;
          pose_msg.pose = det3d.bbox.center;
          pose_power_drill_pub->publish(pose_msg);
        }
      }
    }
  }
  detection_pub->publish(detections);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gazebo_object_publisher");
  ros::NodeHandle nh;
  detection_pub = std::make_shared<ros::Publisher>(nh.advertise<vision_msgs::Detection3DArray>("dope/detected_objects", 10));
  pose_power_drill_pub = std::make_shared<ros::Publisher>(nh.advertise<geometry_msgs::PoseStamped>("/mobipick/dope/pose_power_drill_with_grip", 10));
  pose_table_pub = std::make_shared<ros::Publisher>(nh.advertise<geometry_msgs::PoseStamped>("/mobipick/dope/pose_table", 10));
  ros::Subscriber sub = nh.subscribe("/gazebo/link_states", 10, gazebo_cb);

  ros::spin();
  return 0;
}
