#!/usr/bin/env python3
import math
import numpy as np
from transformations import quaternion_from_euler, euler_from_quaternion, quaternion_from_matrix
import os
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import argparse
from nav_msgs.msg import Odometry
import threading
from std_srvs.srv import Trigger
from data_msgs.msg import TeleopStatus
import time


current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

os.environ['MKL_NUM_THREADS'] = '1'
os.environ['NUMEXPR_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'


def matrix_to_xyzrpy(matrix):
    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]
    roll = math.atan2(matrix[2, 1], matrix[2, 2])
    pitch = math.asin(-matrix[2, 0])
    yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    return [x, y, z, roll, pitch, yaw]


def create_transformation_matrix(x, y, z, roll, pitch, yaw):
    transformation_matrix = np.eye(4)
    A = np.cos(yaw)
    B = np.sin(yaw)
    C = np.cos(pitch)
    D = np.sin(pitch)
    E = np.cos(roll)
    F = np.sin(roll)
    DE = D * E
    DF = D * F
    transformation_matrix[0, 0] = A * C
    transformation_matrix[0, 1] = A * DF - B * E
    transformation_matrix[0, 2] = B * F + A * DE
    transformation_matrix[0, 3] = x
    transformation_matrix[1, 0] = B * C
    transformation_matrix[1, 1] = A * E + B * DF
    transformation_matrix[1, 2] = B * DE - A * F
    transformation_matrix[1, 3] = y
    transformation_matrix[2, 0] = -D
    transformation_matrix[2, 1] = C * F
    transformation_matrix[2, 2] = C * E
    transformation_matrix[2, 3] = z
    transformation_matrix[3, 0] = 0
    transformation_matrix[3, 1] = 0
    transformation_matrix[3, 2] = 0
    transformation_matrix[3, 3] = 1
    return transformation_matrix


class RosOperator(Node):
    def __init__(self, args):
        super().__init__('teleop_piper_publisher')
        self.args = args
        self.localization_pose_subscriber = None
        self.arm_end_pose_subscriber = None

        self.arm_end_pose_ctrl_publisher = None

        self.localization_pose_matrix = None
        self.arm_end_pose_matrix = None

        self.refresh_localization_pose = True
        self.refresh_arm_end_pose = True

        self.thread = None
        self.stop_thread = False

        self.status_srv = None
        self.status = False
        self.init_ros()

    def localization_pose_callback(self, msg):
        roll, pitch, yaw = euler_from_quaternion((msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))
        matrix = create_transformation_matrix(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, roll, pitch, yaw)
        if self.refresh_localization_pose:
            self.refresh_localization_pose = False
            self.localization_pose_matrix = matrix
        if self.arm_end_pose_matrix is not None and self.status:
            pose_xyzrpy = matrix_to_xyzrpy(np.dot(self.arm_end_pose_matrix, np.dot(np.linalg.inv(self.localization_pose_matrix), matrix)))
            pose_msg = PoseStamped()
            pose_msg.header = Header()
            pose_msg.header.frame_id = "map"
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.pose.position.x = pose_xyzrpy[0]
            pose_msg.pose.position.y = pose_xyzrpy[1]
            pose_msg.pose.position.z = pose_xyzrpy[2]
            q = quaternion_from_euler(pose_xyzrpy[3], pose_xyzrpy[4], pose_xyzrpy[5])
            pose_msg.pose.orientation.x = pose_xyzrpy[3]  # q[0]
            pose_msg.pose.orientation.y = pose_xyzrpy[4]  # q[1]
            pose_msg.pose.orientation.z = pose_xyzrpy[5]  # q[2]
            pose_msg.pose.orientation.w = 0.0  # q[3]
            self.arm_end_pose_ctrl_publisher.publish(pose_msg)
            status_msg = TeleopStatus()
            status_msg.quit = False
            status_msg.fail = False
            self.teleop_status_publisher.publish(status_msg)

    def arm_end_pose_callback(self, msg):
        roll, pitch, yaw = euler_from_quaternion((msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))
        matrix = create_transformation_matrix(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, roll, pitch, yaw)
        if self.refresh_arm_end_pose:
            self.refresh_arm_end_pose = False
            self.arm_end_pose_matrix = matrix

    def status_changing(self):
        self.refresh_localization_pose = True
        self.refresh_arm_end_pose = True
        while rclpy.ok():
            if self.stop_thread:
                self.status = False
                self.refresh_localization_pose = False
                self.refresh_arm_end_pose = False
                break
            if not self.refresh_localization_pose and not self.refresh_arm_end_pose:
                print("start")
                self.status = True
                break
            else:
                status_msg = TeleopStatus()
                status_msg.quit = False
                status_msg.fail = True
                print("wait")
                self.teleop_status_publisher.publish(status_msg)
            time.sleep(0.1)

    def teleop_trigger_callback(self, request, response):
        if self.status:
            self.status = False
            status_msg = TeleopStatus()
            status_msg.quit = True
            status_msg.fail = False
            self.teleop_status_publisher.publish(status_msg)
            print("close")
        else:
            if self.thread is None or not self.thread.is_alive():
                self.stop_thread = False
                self.thread = threading.Thread(target=self.status_changing)
                self.thread.start()
            else:
                self.stop_thread = True
                self.thread.join()
                self.status = False
                status_msg = TeleopStatus()
                status_msg.quit = True
                status_msg.fail = False
                self.teleop_status_publisher.publish(status_msg)
                print("close")
        return response

    def init_ros(self):
        self.declare_parameter('index_name', "")
        self.args.index_name = self.get_parameter('index_name').get_parameter_value().string_value
        self.localization_pose_subscriber = self.create_subscription(PoseStamped, f'/pika_pose{self.args.index_name}', self.localization_pose_callback, 1)
        self.arm_end_pose_subscriber = self.create_subscription(PoseStamped, f'/piper_FK{self.args.index_name}/urdf_end_pose_orient', self.arm_end_pose_callback, 1)
        self.arm_end_pose_ctrl_publisher = self.create_publisher(PoseStamped, f'/piper_IK{self.args.index_name}/ctrl_end_pose', 1)
        self.teleop_status_publisher = self.create_publisher(TeleopStatus, f'/teleop_status{self.args.index_name}', 1)
        self.status_srv = self.create_service(Trigger, f'/teleop_trigger{self.args.index_name}', self.teleop_trigger_callback)


def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--index_name', action='store', type=str, help='index_name',
                        default="", required=False)
    # args = parser.parse_args()
    args, unknown = parser.parse_known_args()
    return args


def main():
    args = get_arguments()
    rclpy.init()
    ros_operator = RosOperator(args)
    rclpy.spin(ros_operator)


if __name__ == "__main__":
    main()
