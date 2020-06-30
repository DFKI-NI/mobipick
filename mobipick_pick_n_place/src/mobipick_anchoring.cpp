#include "mobipick_pick_n_place/mobipick_anchoring.h"

MobipickAnchoring::MobipickAnchoring(ros::NodeHandle &nh) : node_(&nh)
{
  
  //detection_pub_ = nh_.advertise<vision_msgs::Detection3DArray>("anchored_objects", 10);
  detection_sub_ = node_->subscribe("/mobipick/dope/detected_objects", 10, &MobipickAnchoring::anchoringCB, this);
  anchoring_server_ = node_->advertiseService("anchoring/attach_power_drill", &MobipickAnchoring::attachCB, this);
  
  // set estimated objetcs TODO: Define by ros parameter in launch file
  power_drill_.found = false;
  power_drill_.is_observable = false;
  power_drill_.is_attached = false;
  power_drill_.object_name = "/mobipick/power_drill";
  power_drill_.id = ObjectID::POWER_DRILL;
  power_drill_.frame = power_drill_.object_name;
  detected_objects_.push_back(power_drill_);
}

void MobipickAnchoring::anchoringCB(const vision_msgs::Detection3DArrayConstPtr &detection_msgs)
{
  this->power_drill_.is_observable = false;
  tf::Transform transform;
  tf::Quaternion q;

  // search in objects for power_drill
  for (auto &&det3d : detection_msgs->detections)
  {
    
    if (det3d.results.empty())
    {
      ROS_ERROR("Detections3D message has empty results!");
    }

    
    // Power drill is not attached and found in detection
    if (det3d.results[0].id == ObjectID::POWER_DRILL && !power_drill_.is_attached)
    {
      // Power drill is seen for the first time
      if (!power_drill_.found)
      {
        power_drill_.found = true;
        ROS_INFO_STREAM("Powerdrill found in frame: \n" << detection_msgs->header.frame_id);
        ROS_INFO_STREAM("with detected pose:\n" << det3d.results[0].pose.pose);
      }

      power_drill_.is_observable = true;

      //set tf origin in specific camera frame
      power_drill_.parent_frame = detection_msgs->header.frame_id;
      transform.setOrigin(tf::Vector3(det3d.results[0].pose.pose.position.x, det3d.results[0].pose.pose.position.y,
                                      det3d.results[0].pose.pose.position.z));
      q.setValue(det3d.results[0].pose.pose.orientation.x, det3d.results[0].pose.pose.orientation.y,
                 det3d.results[0].pose.pose.orientation.z, det3d.results[0].pose.pose.orientation.w);
      transform.setRotation(q);
      power_drill_.transform = transform;
    }
  }

  // broadcast transformation if powerdrill is found
  if (power_drill_.found)
  {
    // powerdrill is not seen or observable directly
    if (!power_drill_.is_observable)
    {
      ROS_INFO("Power Drill not observed.");
      //auto attach powerdrill permanent to odom comp 
      if(!power_drill_.is_attached)
      {
        power_drill_.is_attached = true;
        power_drill_.parent_frame = "/mobipick/odom_comb";
        tf::StampedTransform transform_st;
        try
        {
          tf_listener_.lookupTransform(power_drill_.parent_frame, power_drill_.frame, ros::Time(0), transform_st);
        }
        catch (tf::TransformException ex)
        {
          ROS_ERROR("%s", ex.what());
          ROS_INFO("Transformation not found!");
        }
        transform.setOrigin(transform_st.getOrigin());
        transform.setRotation(transform_st.getRotation());
        power_drill_.transform = transform;
      }


    }
    
    //broadcast transform
    detection_br_.sendTransform(
        tf::StampedTransform(power_drill_.transform, ros::Time::now(), power_drill_.parent_frame, power_drill_.frame));
  }
}

// manual attachment/detachment to specific frame, eg. in gripping scenario
bool MobipickAnchoring::attachCB(mobipick_pick_n_place::AnchoringSrv::Request &req, mobipick_pick_n_place::AnchoringSrv::Response &res)
{
  // if request is empty detach object from specific frame --> wait for new detection or auto-attach
  // ToDo: apply default attachment
  if (req.frame_id.empty())
  {
    ROS_INFO_STREAM("Detached object");
    power_drill_.is_attached=false;
    res.attached = false;
    return 1;
  }

  // set new parent frame and broadcast it's transform
  power_drill_.parent_frame = req.frame_id;
  tf::StampedTransform transform_st;
  tf::Transform transform;
  
  try
  {
    tf_listener_.lookupTransform(power_drill_.parent_frame, power_drill_.frame, ros::Time(0), transform_st);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ROS_INFO("Transformation not found!");
    res.attached = false;
    return 0;
  }
  
  transform.setOrigin(transform_st.getOrigin());
  transform.setRotation(transform_st.getRotation());
  power_drill_.transform = transform;
  power_drill_.is_attached = true;
  res.attached = true;
  ROS_INFO_STREAM("Attached to frame " << power_drill_.parent_frame);
  return 1;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mobipick_anchoring");
  ros::NodeHandle nh;

  MobipickAnchoring mobipick_anchoring(nh);

  ros::spin();

  return 0;
}
