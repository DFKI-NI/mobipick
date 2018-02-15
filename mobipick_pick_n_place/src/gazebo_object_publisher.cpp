#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <vision_msgs/Detection3DArray.h>
#include "mobipick_pick_n_place/fake_object_recognition.h"

std::shared_ptr<ros::Publisher> detection_pub;

void gazebo_cb(const gazebo_msgs::LinkStatesConstPtr msg)
{
  // input topic has 1000 Hz, output topic should have 10 Hz
  static size_t counter = 0;
  counter = (counter + 1) % 100;
  if (counter != 0)
    return;

  vision_msgs::Detection3DArray detections;

  detections.header.stamp = ros::Time::now();
  detections.header.frame_id = "map";

  assert(msg->name.size() == msg->pose.size());
  for (size_t i = 0; i < msg->name.size(); ++i)
  {
    if (msg->name[i] == "table_1::table_top_link")
    {
      // add table
      vision_msgs::Detection3D det3d;
      det3d.header = detections.header;
      det3d.bbox.size.x = 1.0;
      det3d.bbox.size.y = 2.0;
      det3d.bbox.size.z = 1.0;
      det3d.bbox.center = msg->pose[i];
      det3d.bbox.center.position.z += det3d.bbox.size.z / 2.0;
      det3d.results.resize(1);
      det3d.results[0].id = ObjectID::TABLE;
      det3d.results[0].pose.pose = det3d.bbox.center;
      det3d.results[0].score = 1.0;
      detections.detections.push_back(det3d);
    }
    if (msg->name[i] == "cokecan_1::coke_can")
    {
      // add coke_can
      vision_msgs::Detection3D det3d;
      det3d.header = detections.header;
      det3d.bbox.size.x = 0.067;
      det3d.bbox.size.y = 0.067;
      det3d.bbox.size.z = 0.1239;
      det3d.bbox.center = msg->pose[i];
      det3d.bbox.center.position.z += det3d.bbox.size.z / 2.0;
      det3d.results.resize(1);
      det3d.results[0].id = ObjectID::COKE_CAN;
      det3d.results[0].pose.pose = det3d.bbox.center;
      det3d.results[0].score = 1.0;
      detections.detections.push_back(det3d);
    }
  }
  detection_pub->publish(detections);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gazebo_object_publisher");
  ros::NodeHandle nh;
  detection_pub = std::make_shared<ros::Publisher>(nh.advertise<vision_msgs::Detection3DArray>("detected_objects", 10));
  ros::Subscriber sub = nh.subscribe("/gazebo/link_states", 10, gazebo_cb);

  ros::spin();
  return 0;
}
