#include <ros/ros.h>
#include <vision_msgs/Detection3DArray.h>
#include "mobipick_pick_n_place/fake_object_recognition.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "static_object_publisher");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh;
  ros::Publisher detection_pub = nh.advertise<vision_msgs::Detection3DArray>("detected_objects", 10);

  vision_msgs::Detection3DArray detections;

  detections.header.stamp = ros::Time::now();
  detections.header.frame_id = "odom_comb";

  {
    // add table
    vision_msgs::Detection3D det3d;
    det3d.header = detections.header;
    det3d.bbox.size.x = 1.0;
    det3d.bbox.size.y = 2.0;
    det3d.bbox.size.z = 1.0;
    det3d.bbox.center.position.x = 1.0;
    det3d.bbox.center.position.y = 0.0;
    det3d.bbox.center.position.z = det3d.bbox.size.z / 2.0;
    det3d.bbox.center.orientation.w = 1.0;
    det3d.results.resize(1);
    det3d.results[0].id = ObjectID::TABLE;
    det3d.results[0].pose.pose = det3d.bbox.center;
    det3d.results[0].score = 1.0;
    detections.detections.push_back(det3d);
  }

  {
    // add coke_can
    vision_msgs::Detection3D det3d;
    det3d.header = detections.header;
    det3d.bbox.size.x = 0.067;
    det3d.bbox.size.y = 0.067;
    det3d.bbox.size.z = 0.1239;
    det3d.bbox.center.position.x = 1.05;
    det3d.bbox.center.position.y = -0.25;
    det3d.bbox.center.position.z = 1.0 + det3d.bbox.size.z / 2.0;
    det3d.bbox.center.orientation.w = 1.0;
    det3d.results.resize(1);
    det3d.results[0].id = ObjectID::COKE_CAN;
    det3d.results[0].pose.pose = det3d.bbox.center;
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
