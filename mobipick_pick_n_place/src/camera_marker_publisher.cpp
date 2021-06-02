#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>

// camera field of view
static const double HFOV = 60.0 * M_PI / 180.0;
static const double VFOV = 45.0 * M_PI / 180.0;
static const double RANGE_MIN = 0.35;
static const double RANGE_MAX = 1.0;
static const std::string CAMERA_FRAME = "gripper_astra_depth_frame";

static tf::Vector3 sphericalToCartesian(const double& range, const double& elevation, const double& azimuth)
{
  using namespace std;

  // hack: we don't really want spherical coordinates; we want "range" to be the distance from the camera (= the x coordinate)
  double range_adjusted = range / (cos(elevation) * cos(azimuth));

  return range_adjusted * tf::Vector3(cos(elevation) * cos(azimuth), cos(elevation) * sin(azimuth), sin(elevation));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_marker_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  ros::Publisher pub = nh.advertise<visualization_msgs::MarkerArray>("camera_markers", 10);

  visualization_msgs::MarkerArray markers;

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.005;
  nh_priv.param<std::string>("camera_frame", marker.header.frame_id, CAMERA_FRAME);
  marker.frame_locked = 1;
  double lr = HFOV / 2.0;
  double td = VFOV / 2.0;

  /* ********************** white frustum ********************** */
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.id = 0;
  marker.ns = marker.header.frame_id;
  // p_ftl = far top left etc.
  geometry_msgs::Point p_ftl, p_fbl, p_fbr, p_ftr;
  tf::pointTFToMsg(sphericalToCartesian(RANGE_MAX, td, lr), p_ftl);
  tf::pointTFToMsg(sphericalToCartesian(RANGE_MAX, -td, lr), p_fbl);
  tf::pointTFToMsg(sphericalToCartesian(RANGE_MAX, -td, -lr), p_fbr);
  tf::pointTFToMsg(sphericalToCartesian(RANGE_MAX, td, -lr), p_ftr);
  geometry_msgs::Point p_ntl, p_nbl, p_nbr, p_ntr;
  tf::pointTFToMsg(sphericalToCartesian(RANGE_MIN, td, lr), p_ntl);
  tf::pointTFToMsg(sphericalToCartesian(RANGE_MIN, -td, lr), p_nbl);
  tf::pointTFToMsg(sphericalToCartesian(RANGE_MIN, -td, -lr), p_nbr);
  tf::pointTFToMsg(sphericalToCartesian(RANGE_MIN, td, -lr), p_ntr);

  // far plane
  marker.points.push_back(p_ftl);
  marker.points.push_back(p_fbl);

  marker.points.push_back(p_fbl);
  marker.points.push_back(p_fbr);

  marker.points.push_back(p_fbr);
  marker.points.push_back(p_ftr);

  marker.points.push_back(p_ftr);
  marker.points.push_back(p_ftl);

  // near plane
  marker.points.push_back(p_ntl);
  marker.points.push_back(p_nbl);

  marker.points.push_back(p_nbl);
  marker.points.push_back(p_nbr);

  marker.points.push_back(p_nbr);
  marker.points.push_back(p_ntr);

  marker.points.push_back(p_ntr);
  marker.points.push_back(p_ntl);

  // connections
  marker.points.push_back(p_ftl);
  marker.points.push_back(p_ntl);

  marker.points.push_back(p_fbl);
  marker.points.push_back(p_nbl);

  marker.points.push_back(p_fbr);
  marker.points.push_back(p_nbr);

  marker.points.push_back(p_ftr);
  marker.points.push_back(p_ntr);

  markers.markers.push_back(marker);

  /* ********************** magenta pyramid ********************** */
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.id = 1;

  marker.points.clear();

  marker.points.emplace_back();
  marker.points.push_back(p_ntl);

  marker.points.emplace_back();
  marker.points.push_back(p_nbl);

  marker.points.emplace_back();
  marker.points.push_back(p_nbr);

  marker.points.emplace_back();
  marker.points.push_back(p_ntr);

  markers.markers.push_back(marker);

  ros::Rate rate(1.0);
  while (ros::ok())
  {
    marker.header.stamp = ros::Time::now();
    pub.publish(markers);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
