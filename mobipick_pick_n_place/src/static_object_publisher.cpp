#include <ros/ros.h>
#include <vision_msgs/Detection3DArray.h>
#include "mobipick_pick_n_place/fake_object_recognition.h"
#include <eigen_conversions/eigen_msg.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "static_object_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  std::string tf_prefix = "mobipick";
  std::string param_path;
  if (nh_priv.searchParam("tf_prefix", param_path))
    nh.getParam(param_path, tf_prefix);

  // ensure tf_prefix ends with exactly 1 '/' if nonempty, or "" if empty
  tf_prefix = tf_prefix.erase(tf_prefix.find_last_not_of('/') + 1) + "/";
  if (tf_prefix.length() == 1)
    tf_prefix = "";

  ros::Publisher detection_pub = nh.advertise<vision_msgs::Detection3DArray>("dope/detected_objects", 10);

  vision_msgs::Detection3DArray detections;

  detections.header.stamp = ros::Time::now();
  detections.header.frame_id = tf_prefix + "base_link";

  {
    // add table
    geometry_msgs::Pose object_pose_msg;
    object_pose_msg.position.x = 0.11;
    object_pose_msg.position.y = -0.84;
    object_pose_msg.position.z = 0.0;
    object_pose_msg.orientation.z = -1.0;

    vision_msgs::Detection3D det3d;
    det3d.header = detections.header;
    det3d.bbox.size.x = 0.70;
    det3d.bbox.size.y = 0.70;
    det3d.bbox.size.z = 0.73;

    // shift to center of bbox
    Eigen::Isometry3d object_pose;
    tf::poseMsgToEigen(object_pose_msg, object_pose);
    Eigen::Isometry3d object_to_bbox = Eigen::Isometry3d::Identity();
    object_to_bbox.translation() = Eigen::Vector3d(0.0d, 0.0d, det3d.bbox.size.z / 2.0d);
    tf::poseEigenToMsg((object_pose * object_to_bbox), det3d.bbox.center);

    det3d.results.resize(1);
    det3d.results[0].id = ObjectID::TABLE;
    det3d.results[0].pose.pose = object_pose_msg;
    det3d.results[0].score = 1.0;
    detections.detections.push_back(det3d);
  }

  //  {
  //    // add coke_can
  //    geometry_msgs::Pose object_pose_msg;
  //    object_pose_msg.position.x = 1.05;
  //    object_pose_msg.position.y = -0.25;
  //    object_pose_msg.position.z = 1.0;
  //    object_pose_msg.orientation.w = 1.0;
  //
  //    vision_msgs::Detection3D det3d;
  //    det3d.header = detections.header;
  //    det3d.bbox.size.x = 0.067;
  //    det3d.bbox.size.y = 0.067;
  //    det3d.bbox.size.z = 0.1239;
  //
  //    // shift to center of bbox
  //    Eigen::Isometry3d object_pose;
  //    tf::poseMsgToEigen(object_pose_msg, object_pose);
  //    Eigen::Isometry3d object_to_bbox = Eigen::Isometry3d::Identity();
  //    object_to_bbox.translation() = Eigen::Vector3d(0.0d, 0.0d, det3d.bbox.size.z / 2.0d);
  //    tf::poseEigenToMsg((object_pose * object_to_bbox), det3d.bbox.center);
  //
  //    det3d.results.resize(1);
  //    det3d.results[0].id = ObjectID::COKE_CAN;
  //    det3d.results[0].pose.pose = object_pose_msg;
  //    det3d.results[0].score = 1.0;
  //    detections.detections.push_back(det3d);
  //  }

  {
    // add power_drill_with_grip
    geometry_msgs::Pose object_pose_msg;
    object_pose_msg.position.x = 0.11;
    object_pose_msg.position.y = -0.84;
    object_pose_msg.position.z = 0.83;
    object_pose_msg.orientation.x = -0.6194878998936445;
    object_pose_msg.orientation.y = -0.344329398453028;
    object_pose_msg.orientation.z = 0.3426231915182013;
    object_pose_msg.orientation.w = 0.6166695678239843;

    vision_msgs::Detection3D det3d;
    det3d.header = detections.header;
    det3d.bbox.size.x = 0.18002399444580078;
    det3d.bbox.size.y = 0.22317399978637695;
    det3d.bbox.size.z = 0.082321996688842773;

    // shift to center of bbox
    Eigen::Isometry3d object_pose;
    tf::poseMsgToEigen(object_pose_msg, object_pose);
    Eigen::Isometry3d object_to_bbox = Eigen::Isometry3d::Identity();
    object_to_bbox.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
    tf::poseEigenToMsg((object_pose * object_to_bbox), det3d.bbox.center);

    det3d.results.resize(1);
    det3d.results[0].id = ObjectID::POWER_DRILL;
    det3d.results[0].pose.pose = object_pose_msg;
    det3d.results[0].score = 1.0;
    detections.detections.push_back(det3d);
  }

  ros::Rate rate(10.0);
  while (ros::ok())
  {
    detections.header.stamp = ros::Time::now();
    detection_pub.publish(detections);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
