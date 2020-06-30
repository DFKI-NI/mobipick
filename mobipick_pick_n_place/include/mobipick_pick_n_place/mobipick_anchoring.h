#ifndef MOBIPICK_PICK_N_PLACE_MOBIPICK_ANCHORING_H
#define MOBIPICK_PICK_N_PLACE_MOBIPICK_ANCHORING_H

#include <iostream>
#include <ros/ros.h>
#include <vision_msgs/Detection3DArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <mobipick_pick_n_place/AnchoringSrv.h>

class MobipickAnchoring 
{
public:
    MobipickAnchoring(ros::NodeHandle &nh);


private:

    ros::NodeHandle *node_;

    //communication
    ros::Publisher detection_pub_;
    ros::Subscriber detection_sub_;
    tf::TransformBroadcaster detection_br_;
    tf::TransformListener tf_listener_;
    ros::ServiceServer anchoring_server_;
    
    //callbacks
    void anchoringCB(const vision_msgs::Detection3DArrayConstPtr &detection_msgs);
    bool attachCB(mobipick_pick_n_place::AnchoringSrv::Request &req, mobipick_pick_n_place::AnchoringSrv::Response &res);
    
    //anchoring
    enum ObjectID
    {
    GETRIEBELAGER = 1, POWER_DRILL = 2, TABLE = 100, COKE_CAN = 101
    };
    struct DetectedObject_
    {
        bool found;
        std::string object_name;
        ObjectID id;
        vision_msgs::Detection3D det3D;
        bool is_observable;
        bool is_attached;
        std::string frame;
        std::string parent_frame;
        tf::Transform transform;
    };
    std::vector<DetectedObject_> detected_objects_;

    // Object 2 PowerDrill
    DetectedObject_ power_drill_;
    
    
};

#endif //MOBIPICK_PICK_N_PLACE_MOBIPICK_ANCHORING_H