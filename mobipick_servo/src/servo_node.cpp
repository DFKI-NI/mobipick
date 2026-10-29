// MoveIt Servo for Mobipick: moveit_servo's servo_server with a tf buffer for the planning scene monitor.
//
// moveit_servo 1.1.16's servo_server builds its PlanningSceneMonitor without tf, which suits fixed-base arms. Mobipick's
// SRDF has a floating virtual joint map -> base_footprint that only tf can fill, so there the robot state stays
// incomplete ("Missing mobipick/virtual_joint"), the base sits at the map origin, the collision check sees phantom
// contacts and servo scales every command to ~0 (sim 2026-09-29). This node is servo_server plus that tf buffer.
//
// Its scene comes from move_group's monitored_planning_scene without the octomap: servo checks distances, which is slow
// against octree voxels (loop overruns of 0.05-0.1 s), and in the sim the octomap holds phantom voxels of the arm itself
// (the self filter misses it), so servo decelerated "close to a collision" all the time. Boxes (tables, pose
// selector objects) and the robot itself stay in the check.
#include <memory>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_servo/servo.h>
#include <octomap_msgs/OctomapWithPose.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "servo_server");
  ros::AsyncSpinner spinner(8);
  spinner.start();
  ros::NodeHandle nh("~");

  auto tf_buffer = std::make_shared<tf2_ros::Buffer>();
  tf2_ros::TransformListener tf_listener(*tf_buffer);
  auto planning_scene_monitor =
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description", tf_buffer);
  if (!planning_scene_monitor->getPlanningScene())
  {
    ROS_ERROR("mobipick_servo: could not set up the planning scene monitor");
    return 1;
  }
  // move_group's scene without its octomap (an empty octomap in the message removes it from servo's scene)
  ros::NodeHandle ns_nh;
  ros::Subscriber scene_sub = ns_nh.subscribe<moveit_msgs::PlanningScene>(
      "move_group/monitored_planning_scene", 2, [planning_scene_monitor](const moveit_msgs::PlanningSceneConstPtr& msg) {
        moveit_msgs::PlanningScene scene = *msg;
        scene.world.octomap = octomap_msgs::OctomapWithPose();
        planning_scene_monitor->newPlanningSceneMessage(scene);
      });
  planning_scene_monitor->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC, false /* no octomap */);
  planning_scene_monitor->startStateMonitor();

  moveit_servo::Servo servo(nh, planning_scene_monitor);
  servo.start();
  ros::waitForShutdown();
  servo.setPaused(true);
  return 0;
}
