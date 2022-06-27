#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <vision_msgs/Detection3DArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/PoseStamped.h>
#include "mobipick_pick_n_place/fake_object_recognition.h"
#include <eigen_conversions/eigen_msg.h>

std::shared_ptr<ros::Publisher> detection_pub;
std::shared_ptr<ros::Publisher> pose_power_drill_pub;
std::shared_ptr<ros::Publisher> pose_table_pub;
std::shared_ptr<ros::Publisher> marker_pub;

std::string robot_name_ = "mobipick";
std::string tf_prefix_ = "mobipick";

void gazebo_cb(const gazebo_msgs::LinkStatesConstPtr& msg)
{
  // input topic has 1000 Hz, output topic should have 10 Hz
  static size_t counter = 0;
  counter = (counter + 1) % 100;
  if (counter != 0)
    return;

  vision_msgs::Detection3DArray detections;

  detections.header.stamp = ros::Time::now();
  detections.header.frame_id = tf_prefix_ + "base_link";

  assert(msg->name.size() == msg->pose.size());
  bool base_link_found = false;
  Eigen::Isometry3d mobipick_pose;
  for (size_t i = 0; i < msg->name.size(); ++i)
  {
    // TODO: Change to tf listener?
    if (msg->name[i] == robot_name_ + "::" + tf_prefix_ + "base_footprint" && !base_link_found)
    {
      tf::poseMsgToEigen(msg->pose[i], mobipick_pose);
      mobipick_pose = mobipick_pose.inverse();
      base_link_found = true;
      break;
    }
  }
  if (!base_link_found)
  {
    ROS_ERROR_THROTTLE(1.0, "Could not find base link!");
    return;
  }
  for (size_t i = 0; i < msg->name.size(); ++i)
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
      object_to_bbox.translation() = Eigen::Vector3d(0.0, 0.0, det3d.bbox.size.z / 2.0);
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
    else if (msg->name[i] == "cokecan_1::coke_can")
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
      object_to_bbox.translation() = Eigen::Vector3d(0.0, 0.0, det3d.bbox.size.z / 2.0);
      tf::poseEigenToMsg((mobipick_pose * object_pose * object_to_bbox), det3d.bbox.center);

      det3d.results.resize(1);
      det3d.results[0].id = ObjectID::COKE_CAN;
      tf::poseEigenToMsg((mobipick_pose * object_pose), det3d.results[0].pose.pose);
      det3d.results[0].score = 1.0;
      detections.detections.push_back(det3d);
    }
    else if (msg->name[i] == "powerdrill_1::powerdrill")
    {
      // add powerdrill
      vision_msgs::Detection3D det3d;
      det3d.header = detections.header;
      det3d.bbox.size.x = 0.18436100006103516;
      det3d.bbox.size.y = 0.18683599472045898;
      det3d.bbox.size.z = 0.057192001342773438;

      // shift to center of bbox and rotate coordinate frames to match dope detection
      Eigen::Isometry3d object_pose;
      tf::poseMsgToEigen(msg->pose[i], object_pose);

      Eigen::Isometry3d object_to_bbox = Eigen::Isometry3d::Identity();
      object_to_bbox.translation() = Eigen::Vector3d(-0.046, 0.01055, 0.02545);

      // rotate to dope orientation
      Eigen::Isometry3d bbox_center_rotated = Eigen::Isometry3d::Identity();
      bbox_center_rotated.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(1.0, 0.0, 0.0)));
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
    else if (msg->name[i] == "powerdrill_with_grip_1::powerdrill_with_grip")
    {
      // add powerdrill
      vision_msgs::Detection3D det3d;
      det3d.header = detections.header;
      det3d.bbox.size.x = 0.18002399444580078;
      det3d.bbox.size.y = 0.22317399978637695;
      det3d.bbox.size.z = 0.082321996688842773;

      // transform from gazebo world frame to base_link
      Eigen::Isometry3d object_pose;
      tf::poseMsgToEigen(msg->pose[i], object_pose);
      tf::poseEigenToMsg((mobipick_pose * object_pose), det3d.bbox.center);

      det3d.results.resize(1);
      det3d.results[0].id = ObjectID::POWER_DRILL;
      det3d.results[0].pose.pose = det3d.bbox.center;
      det3d.results[0].score = 1.0;
      detections.detections.push_back(det3d);
      // publish power drill pose seperately
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header = det3d.header;
      pose_msg.pose = det3d.bbox.center;
      pose_power_drill_pub->publish(pose_msg);
    }
  }
  detection_pub->publish(detections);

  // ------------ PUBLISH MARKERS
  // Object markers
  visualization_msgs::MarkerArray markers;

  int32_t id = 0;
  for (auto det : detections.detections)
  {
    std::string name = id_to_string(det.results.at(0).id);

    std_msgs::ColorRGBA color_rgba;
    // TODO:
    // draw_color = self.draw_colors[name]
    // color_rgba.r = draw_color[0] / 255.0
    // color_rgba.g = draw_color[1] / 255.0
    // color_rgba.b = draw_color[2] / 255.0
    color_rgba.r = 152.0 / 255.0;
    color_rgba.g = 78.0 / 255.0;
    color_rgba.b = 163.0 / 255.0;
    color_rgba.a = 1.0;

    // cube marker
    {
      visualization_msgs::Marker marker;
      marker.header = detections.header;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose = det.bbox.center;
      marker.color = color_rgba;
      marker.color.a = 0.4;
      marker.ns = "bboxes";
      marker.id = id;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.scale = det.bbox.size;
      markers.markers.push_back(marker);
    }

    // text marker
    {
      visualization_msgs::Marker marker;
      marker.header = detections.header;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose = det.bbox.center;
      marker.color = color_rgba;
      marker.color.a = 1.0;
      marker.ns = "texts";
      marker.id = id;
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.text = name + " (1.0)";
      markers.markers.push_back(marker);
    }

    // TODO: mesh marker
    // try:
    //     marker = Marker()
    //     marker.header = detection_array.header
    //     marker.action = Marker.ADD
    //     marker.pose = det.bbox.center
    //     marker.color = color_rgba
    //     marker.color.a = 0.7
    //     marker.ns = namespace_prefix + "meshes"
    //     marker.id = i
    //     marker.type = Marker.MESH_RESOURCE
    //     marker.scale.x = self.mesh_scales[name]
    //     marker.scale.y = self.mesh_scales[name]
    //     marker.scale.z = self.mesh_scales[name]
    //     marker.mesh_resource = self.mesh_urls[name]
    //     markers.markers.append(marker)
    // except KeyError:
    //     # user didn't specify self.meshes[name], so don't publish marker
    //     pass

    id++;
  }

  static size_t prev_num_detections = 0;
  for (size_t i = detections.detections.size(); i < prev_num_detections; i++)
  {
    {
      visualization_msgs::Marker marker;
      marker.action = visualization_msgs::Marker::DELETE;
      marker.ns = "bboxes";
      marker.id = i;
      markers.markers.push_back(marker);
    }
    {
      visualization_msgs::Marker marker;
      marker.action = visualization_msgs::Marker::DELETE;
      marker.ns = "texts";
      marker.id = i;
      markers.markers.push_back(marker);
    }
    {
      visualization_msgs::Marker marker;
      marker.action = visualization_msgs::Marker::DELETE;
      marker.ns = "meshes";
      marker.id = i;
      markers.markers.push_back(marker);
    }
  }
  prev_num_detections = detections.detections.size();

  marker_pub->publish(markers);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gazebo_object_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  std::string param_path;
  if (nh_priv.searchParam("robot_name", param_path))
    nh.getParam(param_path, robot_name_);
  if (nh_priv.searchParam("tf_prefix", param_path))
    nh.getParam(param_path, tf_prefix_);

  nh_priv.param<std::string>("robot_name", robot_name_, "mobipick");
  nh_priv.param<std::string>("tf_prefix", tf_prefix_, "mobipick");

  // ensure tf_prefix_ ends with exactly 1 '/' if nonempty, or "" if empty
  tf_prefix_ = tf_prefix_.erase(tf_prefix_.find_last_not_of('/') + 1) + "/";
  if (tf_prefix_.length() == 1)
    tf_prefix_ = "";

  detection_pub =
      std::make_shared<ros::Publisher>(nh.advertise<vision_msgs::Detection3DArray>("dope/detected_objects", 10));
  pose_power_drill_pub =
      std::make_shared<ros::Publisher>(nh.advertise<geometry_msgs::PoseStamped>("dope/pose_power_drill_with_grip", 10));
  pose_table_pub = std::make_shared<ros::Publisher>(nh.advertise<geometry_msgs::PoseStamped>("dope/pose_table", 10));
  marker_pub = std::make_shared<ros::Publisher>(nh.advertise<visualization_msgs::MarkerArray>("dope/markers", 10));
  ros::Subscriber sub = nh.subscribe("/gazebo/link_states", 10, gazebo_cb);

  ros::spin();
  return 0;
}
