#include <ros/ros.h>
#include <vision_msgs/Detection3DArray.h>
#include "mobipick_pick_n_place/fake_object_recognition.h"
#include <eigen_conversions/eigen_msg.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "static_object_publisher");
  ros::NodeHandle nh;
  ros::Publisher detection_pub = nh.advertise<vision_msgs::Detection3DArray>("detected_objects", 10);

  vision_msgs::Detection3DArray detections;

  detections.header.stamp = ros::Time::now();
  detections.header.frame_id = "odom_comb";

  {
    // add table
    geometry_msgs::Pose object_pose_msg;
    object_pose_msg.position.x = 1.1;
    object_pose_msg.position.y = 0.0;
    object_pose_msg.position.z = 0.0;
    object_pose_msg.orientation.w = 1.0;

    vision_msgs::Detection3D det3d;
    det3d.header = detections.header;
    det3d.bbox.size.x = 0.70;
    det3d.bbox.size.y = 0.70;
    det3d.bbox.size.z = 0.94;

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
    // add powerdrill
    geometry_msgs::Pose object_pose_msg;
    object_pose_msg.position.x = 0.90;
    object_pose_msg.position.y = -0.20;
    object_pose_msg.position.z = 1.10;
    object_pose_msg.orientation.x = 0.0;
    object_pose_msg.orientation.y = -0.707106781;
    object_pose_msg.orientation.z = -0.707106781;
    object_pose_msg.orientation.w = 0.0;

    vision_msgs::Detection3D det3d;
    det3d.header = detections.header;
    det3d.bbox.size.x = 0.184208;
    det3d.bbox.size.y = 0.187514;
    det3d.bbox.size.z = 0.057294;

    // shift to center of bbox
    Eigen::Isometry3d object_pose;
    tf::poseMsgToEigen(object_pose_msg, object_pose);
    Eigen::Isometry3d object_to_bbox = Eigen::Isometry3d::Identity();
    object_to_bbox.translation() = Eigen::Vector3d(-0.046d, 0.01055d, 0.02545d);
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
