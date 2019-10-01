#!/usr/bin/env python

import rospy
import rospkg
import threading
import yaml
from pbr_msgs.srv import SaveArmPose, SetArmPose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class PoseTeacher(object):
    def __init__(self):
        rospy.loginfo("initializing...")

        # get a filename to save the poses to
        rospack = rospkg.RosPack()
        package = "mobipick_moveit_config"
        self.filename = rospack.get_path(package) + "/config/teached_poses.yml"

        # subscribe to joint states
        self.latestJointState = None
        self.jointStateLock = threading.Lock()

        self.relevantJoints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.jointPrefix = "mobipick/ur5_"

        self.jointListener = rospy.Subscriber("joint_states", JointState, self.setLatestJointState, queue_size=5)

        # provide a service to save a pose
        self.savePoseServer = rospy.Service("~save_arm_pose", SaveArmPose, self.saveArmPose)

        # provide a service to drive to a pose
        self.driveToPoseServer = rospy.Service("~set_goal_pose", SetArmPose, self.setArmPose)
        # ... which just publishes a desired joint state once ...
        self.pubJointTrajectory = rospy.Publisher("command", JointTrajectory, queue_size=5)

        rospy.loginfo("done.")

    def setLatestJointState(self, jointState):
        with self.jointStateLock:
            self.latestJointState = jointState

    def setArmPose(self, request):
        with self.jointStateLock:
            # read the config file
            try:
                with open(self.filename) as poseFile:
                    poses = yaml.load(poseFile)
            except IOError as err:
                print "Could not open %s to read poses" % self.filename
                print err
                raise

            if not request.name in poses:
                raise Exception("Unknown pose: %s" % request.name)

            # create a JointTrajectory message
            msg = JointTrajectory()
            msg.points.append(JointTrajectoryPoint())
            for key, value in poses[request.name].items():
                msg.joint_names.append(self.jointPrefix + key)
                msg.points[0].positions.append(value)
            msg.points[0].time_from_start.secs = 5

            # publish to drive the arm
            self.pubJointTrajectory.publish(msg)
            return "ok"


    def saveArmPose(self, request):
        with self.jointStateLock:
            if self.latestJointState is None:
                raise Exception("No JointState received yet")

            # read from file
            try:
                with open(self.filename) as poseFile:
                    poses = yaml.load(poseFile)
            except IOError as err:
                print "Could not open %s to read poses" % self.filename
                print err
                poses = {}
            
            joints = {}
            if request.name in poses:
                joints = poses[request.name]

            # update values
            for joint in self.relevantJoints:
                index = self.latestJointState.name.index(self.jointPrefix + joint)
                value = self.latestJointState.position[index]
                joints[joint] = value

            poses[request.name] = joints

            # save to file
            with open(self.filename, 'w') as poseFile:
                yaml.dump(poses, poseFile)

        return "ok"


if __name__ == '__main__':
    rospy.init_node("PoseTeacher")
    pt = PoseTeacher()
    rospy.spin()
