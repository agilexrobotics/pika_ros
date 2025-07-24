import math
import numpy as np
import os
import sys
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import argparse
from nav_msgs.msg import Odometry
import threading
from functools import partial


current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)


class RosOperator:
    def __init__(self, args):
        self.args = args
        self.pose_subscribers = []
        self.pose_publishers = []
        self.joint_state_subscribers = []
        self.joint_state_publishers = []
        self.init_ros()

    def pose_callback(self, index, msg):
        self.pose_publishers[index].publish(msg)

    def joint_state_callback(self, index, msg):
        self.joint_state_publishers[index].publish(msg)

    def init_ros(self):
        rospy.init_node(f'transmit_topics', anonymous=True)
        self.pose_publishers = [rospy.Publisher(self.args.pub_pose_topics[i], PoseStamped, queue_size=1) for i in range(len(self.args.pub_pose_topics))]
        self.joint_state_publishers = [rospy.Publisher(self.args.pub_joint_state_topics[i], JointState, queue_size=1) for i in range(len(self.args.pub_joint_state_topics))]
        self.pose_subscribers = [rospy.Subscriber(self.args.sub_pose_topics[i], PoseStamped, partial(self.pose_callback, i), queue_size=1) for i in range(len(self.args.sub_pose_topics))]
        self.joint_state_subscribers = [rospy.Subscriber(self.args.sub_joint_state_topics[i], JointState, partial(self.joint_state_callback, i), queue_size=1) for i in range(len(self.args.sub_joint_state_topics))]


def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sub_pose_topics', action='store', type=str, help='sub_pose_topics',
                        default=[], required=False)
    parser.add_argument('--pub_pose_topics', action='store', type=str, help='pub_pose_topics',
                        default=[], required=False)
    parser.add_argument('--sub_joint_state_topics', action='store', type=str, help='sub_joint_state_topics',
                        default=["/sensor/gripper_l/joint_states", "/sensor/gripper_r/joint_states"], required=False)
    parser.add_argument('--pub_joint_state_topics', action='store', type=str, help='pub_joint_state_topics',
                        default=["/gripper/joint_states_l", "/gripper/joint_states_r"], required=False)
    args = parser.parse_args()
    return args


def main():
    args = get_arguments()
    ros_operator = RosOperator(args)
    rospy.spin()


if __name__ == "__main__":
    main()
