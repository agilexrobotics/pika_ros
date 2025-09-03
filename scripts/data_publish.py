#!/usr/bin/env python3
# -- coding: UTF-8

import os
import time

import cv2
import numpy as np
import h5py
import argparse

import collections
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField, Imu
from data_msgs.msg import Gripper
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import pcl
import threading
# import ros2_numpy
from sensor_msgs_py import point_cloud2
import yaml
USELIFT = False
if USELIFT:
    from bt_task_msgs.msg import LiftMotorMsg
    from bt_task_msgs.srv import LiftMotorSrv


class RosOperator(Node):
    def __init__(self, args): 
        super().__init__('data_publish')
        self.args = args
        self.bridge = None
        self.camera_color_publishers = []
        self.camera_depth_publishers = []
        self.camera_point_cloud_publishers = []
        self.arm_joint_state_publishers = []
        self.arm_end_pose_publishers = []
        self.localization_pose_publishers = []
        self.gripper_encoder_publishers = []
        self.imu_9axis_publishers = []
        self.lidar_point_cloud_publishers = []
        self.robot_base_vel_publishers = []
        self.lift_motor_publishers = []
        self.init_ros()
        self.thread = threading.Thread(target=self.process_data)
        self.thread.start()

    def init_ros(self):
        self.bridge = CvBridge()
        self.camera_color_publishers = [self.create_publisher(Image, topic, 10) for topic in self.args.camera_color_topics]
        self.camera_depth_publishers = [self.create_publisher(Image, topic, 10) for topic in self.args.camera_depth_topics]
        self.camera_point_cloud_publishers = [self.create_publisher(PointCloud2, topic, 10) for topic in self.args.camera_point_cloud_topics]
        self.arm_joint_state_publishers = [self.create_publisher(JointState, topic, 10) for topic in self.args.arm_joint_state_topics]
        self.arm_end_pose_publishers = [self.create_publisher(PoseStamped, topic, 10) for topic in self.args.arm_end_pose_topics]
        self.localization_pose_publishers = [self.create_publisher(PoseStamped, topic, 10) for topic in self.args.localization_pose_topics]
        self.gripper_encoder_publishers = [self.create_publisher(Gripper, topic, 10) for topic in self.args.gripper_encoder_topics]
        self.imu_9axis_publishers = [self.create_publisher(Imu, topic, 10) for topic in self.args.imu_9axis_topics]
        self.lidar_point_cloud_publishers = [self.create_publisher(PointCloud2, topic, 10) for topic in self.args.lidar_point_cloud_topics]
        self.robot_base_vel_publishers = [self.create_publisher(Twist, topic, 10) for topic in self.args.robot_base_vel_topics]
        if USELIFT:
            self.lift_motor_publishers = [self.create_publisher(LiftMotorMsg, topic, 10) for topic in self.args.lift_motor_topics]
    
    def publish_camera_color(self, index, color):
        msg = self.bridge.cv2_to_imgmsg(color, "bgr8")
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.camera_color_publishers[index].publish(msg)

    def publish_camera_depth(self, index, depth):
        msg = self.bridge.cv2_to_imgmsg(depth, "16UC1")
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.camera_depth_publishers[index].publish(msg)

    def publish_camera_point_cloud(self, index, point_cloud):
        msg = self.pcd_to_msg_rgb(point_cloud)
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.camera_point_cloud_publishers[index].publish(msg)

    def publish_arm_joint_state(self, index, joint_state):
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [f'joint{i}' for i in range(len(joint_state))]
        joint_state_msg.position = [float(v) for v in joint_state]
        self.arm_joint_state_publishers[index].publish(joint_state_msg)

    def publish_arm_end_pose(self, index, end_pose):
        end_pose_msg = PoseStamped()
        end_pose_msg.header = Header()
        end_pose_msg.header.frame_id = "map"
        end_pose_msg.header.stamp = self.get_clock().now().to_msg()
        if self.args.arm_end_pose_orients[index]:
            q = quaternion_from_euler(end_pose[3], end_pose[4], end_pose[5])
            end_pose_msg.pose.orientation.x = q[0]
            end_pose_msg.pose.orientation.y = q[1]
            end_pose_msg.pose.orientation.z = q[2]
            end_pose_msg.pose.orientation.w = q[3]
        else:
            end_pose_msg.pose.position.x = end_pose[0]
            end_pose_msg.pose.position.y = end_pose[1]
            end_pose_msg.pose.position.z = end_pose[2]
            end_pose_msg.pose.orientation.x = end_pose[3]
            end_pose_msg.pose.orientation.y = end_pose[4]
            end_pose_msg.pose.orientation.z = end_pose[5]
            end_pose_msg.pose.orientation.w = end_pose[6]
        self.arm_end_pose_publishers[index].publish(end_pose_msg)

    def publish_localization_pose(self, index, pose):
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = pose[0]
        pose_msg.pose.position.y = pose[1]
        pose_msg.pose.position.z = pose[2]
        q = quaternion_from_euler(pose[3], pose[4], pose[5])
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        self.localization_pose_publishers[index].publish(pose_msg)

    def publish_gripper_encoder(self, index, encoder_angle, encoder_distance):
        gripper_msg = Gripper()
        gripper_msg.header = Header()
        gripper_msg.header.frame_id = "map"
        gripper_msg.header.stamp = self.get_clock().now().to_msg()
        gripper_msg.angle = float(encoder_angle)
        gripper_msg.distance = float(encoder_distance)
        self.gripper_encoder_publishers[index].publish(gripper_msg)

    def publish_imu_9axis(self, index, orientation, angular_velocity, linear_acceleration):
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.frame_id = "map"
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.orientation.x = orientation[0]
        imu_msg.orientation.y = orientation[1]
        imu_msg.orientation.z = orientation[2]
        imu_msg.orientation.w = orientation[3]
        imu_msg.angular_velocity.x = angular_velocity[0]
        imu_msg.angular_velocity.y = angular_velocity[1]
        imu_msg.angular_velocity.z = angular_velocity[2]
        imu_msg.linear_acceleration.x = linear_acceleration[0]
        imu_msg.linear_acceleration.y = linear_acceleration[1]
        imu_msg.linear_acceleration.z = linear_acceleration[2]
        self.imu_9axis_publishers[index].publish(imu_msg)

    def publish_lidar_point_cloud(self, index, point_cloud):
        self.lidar_point_cloud_publishers[index].publish(self.pcd_to_msg(point_cloud))

    def publish_robot_base_vel(self, index, vel):
        vel_msg = Twist()
        vel_msg.linear.x = vel[0]
        vel_msg.linear.y = vel[1]
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = vel[2]
        self.robot_base_vel_publishers[index].publish(vel_msg)

    def publish_lift_motor(self, index, val):
        if USELIFT:
            # rclpy.wait_for_service(self.args.lift_motor_topics[index])
            try:
                client = self.create_client(LiftMotorSrv, self.args.lift_motor_topics[index])
                req = LiftMotorSrv.Request()
                req.val = val
                req.mode = 0
                response = client.call_async(req)
                return response
            except rclpy.ServiceException as e:
                rclpy.logerr("Service call failed: %s" % e)
                return None

    def pcd_to_msg_rgb(self, points):
        header = Header(frame_id="camera", stamp=self.get_clock().now().to_msg())
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
        ]
        pointcloud_msg = point_cloud2.create_cloud(header, fields, points)
        return pointcloud_msg

    def pcd_to_msg(self, points):
        header = Header(frame_id="lidar", stamp=self.get_clock().now().to_msg())
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        pointcloud_msg = point_cloud2.create_cloud(header, fields, points)
        return pointcloud_msg

    def process_data(self):
        episode_dir = os.path.join(self.args.datasetDir, self.args.episodeName)
        data_path = os.path.join(episode_dir, 'data.hdf5')
        if not os.path.exists(data_path):
            data_path = os.path.join(self.args.datasetDir, self.args.episodeName + '.hdf5')
        self.rate = self.create_rate(self.args.publish_rate)
        with h5py.File(data_path, 'r') as root:
            max_action_len = root['size'][()]
            if self.args.publishIndex != -1:
                while rclpy.ok():
                    i = self.args.publishIndex
                    for j in range(len(self.args.camera_color_names)):
                        if root[f'/camera/color/{self.args.camera_color_names[j]}'].ndim == 1:
                            self.publish_camera_color(j, cv2.imread(os.path.join(episode_dir, root[f'/camera/color/{self.args.camera_color_names[j]}'][i].decode('utf-8')), cv2.IMREAD_UNCHANGED))
                        else:
                            self.publish_camera_color(j, root[f'/camera/color/{self.args.camera_color_names[j]}'][i])
                    for j in range(len(self.args.camera_depth_names)):
                        if root[f'/camera/depth/{self.args.camera_depth_names[j]}'].ndim == 1:
                            self.publish_camera_depth(j, cv2.imread(os.path.join(episode_dir, root[f'/camera/depth/{self.args.camera_depth_names[j]}'][i].decode('utf-8')), cv2.IMREAD_UNCHANGED))
                        else:
                            self.publish_camera_depth(j, root[f'/camera/depth/{self.args.camera_depth_names[j]}'][i])
                    for j in range(len(self.args.camera_point_cloud_names)):
                        if f'/lidar/pointCloud/{self.args.camera_point_cloud_names[j]}' in root.keys():
                            if root[f'/camera/pointCloud/{self.args.camera_point_cloud_names[j]}'].ndim == 1 and root[f'/camera/pointCloud/{self.args.camera_point_cloud_names[j]}'][i].decode('utf-8')[-3:] == 'pcd':
                                self.publish_camera_point_cloud(j, pcl.load_XYZRGB(os.path.join(episode_dir, root[f'/camera/pointCloud/{self.args.camera_point_cloud_names[j]}'][i].decode('utf-8'))).to_array())
                            else:
                                if root[f'/camera/pointCloud/{self.args.camera_point_cloud_names[j]}'].ndim == 1 and root[f'/camera/pointCloud/{self.args.camera_point_cloud_names[j]}'][i].decode('utf-8')[-3:] == 'npy':
                                    pc = np.load(os.path.join(episode_dir, root[f'/camera/pointCloud/{self.args.camera_point_cloud_names[j]}'][i].decode('utf-8')))
                                else:
                                    pc = root[f'/camera/pointCloud/{self.args.camera_point_cloud_names[j]}'][i]
                                # rgb = (pc[:, 3]).astype(np.uint32)*(2**16) + (pc[:, 4]).astype(np.uint32)*(2**8) + (pc[:, 5]).astype(np.uint32)
                                # dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.uint32)]
                                # points = np.zeros(rgb.shape[0], dtype=dtype)
                                # points['x'] = pc[:, 0]
                                # points['y'] = pc[:, 1]
                                # points['z'] = pc[:, 2]
                                # points['rgb'] = rgb
                                # # pc[:, 3] = rgb
                                # # rgb = rgb[:, np.newaxis]
                                # # pc = np.concatenate([pc[:, :3], rgb], axis=1).astype(np.float32)
                                # # print(pc)
                                packed_points = []
                                for point in pc:
                                    x, y, z = point[:3]
                                    r, g, b = point[3:]
                                    rgb = (int(r) << 16) | (int(g) << 8) | int(b)
                                    packed_points.append([x, y, z, rgb])
                                self.publish_camera_point_cloud(j, packed_points)
                    for j in range(len(self.args.arm_joint_state_names)):
                        self.publish_arm_joint_state(j, root[f'/arm/jointStatePosition/{self.args.arm_joint_state_names[j]}'][i])
                    for j in range(len(self.args.arm_end_pose_names)):
                        self.publish_arm_end_pose(j, root[f'/arm/endPose/{self.args.arm_end_pose_names[j]}'][i])
                    for j in range(len(self.args.localization_pose_names)):
                        self.publish_localization_pose(j, root[f'/localization/pose/{self.args.localization_pose_names[j]}'][i])
                    for j in range(len(self.args.gripper_encoder_names)):
                        self.publish_gripper_encoder(j, root[f'/gripper/encoderAngle/{self.args.gripper_encoder_names[j]}'][i], root[f'/gripper/encoderDistance/{self.args.gripper_encoder_names[j]}'][i])
                    for j in range(len(self.args.imu_9axis_names)):
                        self.publish_imu_9axis(j, root[f'/imu/9axisOrientation/{self.args.imu_9axis_names[j]}'][i], root[f'/imu/9axisAngularVelocity/{self.args.imu_9axis_names[j]}'][i], root[f'/imu/9axisLinearAcceleration/{self.args.imu_9axis_names[j]}'][i])
                    for j in range(len(self.args.lidar_point_cloud_names)):
                        if f'/lidar/pointCloud/{self.args.lidar_point_cloud_names[j]}' in root.keys():
                            if root[f'/lidar/pointCloud/{self.args.lidar_point_cloud_names[j]}'].ndim == 1 and root[f'/lidar/pointCloud/{self.args.lidar_point_cloud_names[j]}'][i].decode('utf-8')[-3:] == 'pcd':
                                self.publish_lidar_point_cloud(j, pcl.load_XYZI(os.path.join(episode_dir, root[f'/lidar/pointCloud/{self.args.lidar_point_cloud_names[j]}'][i].decode('utf-8'))).to_array())
                            else:
                                if root[f'/lidar/pointCloud/{self.args.lidar_point_cloud_names[j]}'].ndim == 1 and root[f'/lidar/pointCloud/{self.args.lidar_point_cloud_names[j]}'][i].decode('utf-8')[-3:] == 'npy':
                                    pc = np.load(os.path.join(episode_dir, root[f'/lidar/pointCloud/{self.args.lidar_point_cloud_names[j]}'][i].decode('utf-8')))
                                else:
                                    pc = root[f'/lidar/pointCloud/{self.args.lidar_point_cloud_names[j]}'][i]
                                # points = np.zeros(rgb.shape[0], dtype=dtype)
                                # points['x'] = pc[:, 0]
                                # points['y'] = pc[:, 1]
                                # points['z'] = pc[:, 2]
                                packed_points = []
                                for point in pc:
                                    x, y, z = point[:3]
                                    packed_points.append([x, y, z])
                                self.publish_lidar_point_cloud(j, packed_points)
                    for j in range(len(self.args.robot_base_vel_names)):
                        self.publish_robot_base_vel(j, root[f'/robotBase/vel/{self.args.robot_base_vel_names[j]}'][i])
                for j in range(len(self.args.lift_motor_names)):
                    self.ros_operator.publish_lift_motor(j, root[f'/lift/motor/{self.args.lift_motor_names[j]}'][i])
                print("frame:", i)
                self.rate.sleep()
            else:
                for i in range(max_action_len):
                    if not rclpy.ok():
                        return
                    for j in range(len(self.args.camera_color_names)):
                        if root[f'/camera/color/{self.args.camera_color_names[j]}'].ndim == 1:
                            self.publish_camera_color(j, cv2.imread(os.path.join(episode_dir, root[f'/camera/color/{self.args.camera_color_names[j]}'][i].decode('utf-8')), cv2.IMREAD_UNCHANGED))
                        else:
                            self.publish_camera_color(j, root[f'/camera/color/{self.args.camera_color_names[j]}'][i])
                    for j in range(len(self.args.camera_depth_names)):
                        if root[f'/camera/depth/{self.args.camera_depth_names[j]}'].ndim == 1:
                            self.publish_camera_depth(j, cv2.imread(os.path.join(episode_dir, root[f'/camera/depth/{self.args.camera_depth_names[j]}'][i].decode('utf-8')), cv2.IMREAD_UNCHANGED))
                        else:
                            self.publish_camera_depth(j, root[f'/camera/depth/{self.args.camera_depth_names[j]}'][i])
                    for j in range(len(self.args.camera_point_cloud_names)):
                        if f'/lidar/pointCloud/{self.args.camera_point_cloud_names[j]}' in root.keys():
                            if root[f'/camera/pointCloud/{self.args.camera_point_cloud_names[j]}'].ndim == 1 and root[f'/camera/pointCloud/{self.args.camera_point_cloud_names[j]}'][i].decode('utf-8')[-3:] == 'pcd':
                                self.publish_camera_point_cloud(j, pcl.load_XYZRGB(os.path.join(episode_dir, root[f'/camera/pointCloud/{self.args.camera_point_cloud_names[j]}'][i].decode('utf-8'))).to_array())
                            else:
                                if root[f'/camera/pointCloud/{self.args.camera_point_cloud_names[j]}'].ndim == 1 and root[f'/camera/pointCloud/{self.args.camera_point_cloud_names[j]}'][i].decode('utf-8')[-3:] == 'npy':
                                    pc = np.load(os.path.join(episode_dir, root[f'/camera/pointCloud/{self.args.camera_point_cloud_names[j]}'][i].decode('utf-8')))
                                else:
                                    pc = root[f'/camera/pointCloud/{self.args.camera_point_cloud_names[j]}'][i]
                                # rgb = (pc[:, 3]).astype(np.uint32)*(2**16) + (pc[:, 4]).astype(np.uint32)*(2**8) + (pc[:, 5]).astype(np.uint32)
                                # dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.uint32)]
                                # points = np.zeros(rgb.shape[0], dtype=dtype)
                                # points['x'] = pc[:, 0]
                                # points['y'] = pc[:, 1]
                                # points['z'] = pc[:, 2]
                                # points['rgb'] = rgb
                                # # pc[:, 3] = rgb
                                # # rgb = rgb[:, np.newaxis]
                                # # pc = np.concatenate([pc[:, :3], rgb], axis=1).astype(np.float32)
                                # # print(pc)
                                packed_points = []
                                for point in pc:
                                    x, y, z = point[:3]
                                    r, g, b = point[3:]
                                    rgb = (int(r) << 16) | (int(g) << 8) | int(b)
                                    packed_points.append([x, y, z, rgb])
                                self.publish_camera_point_cloud(j, packed_points)
                    for j in range(len(self.args.arm_joint_state_names)):
                        self.publish_arm_joint_state(j, root[f'/arm/jointStatePosition/{self.args.arm_joint_state_names[j]}'][i])
                    for j in range(len(self.args.arm_end_pose_names)):
                        self.publish_arm_end_pose(j, root[f'/arm/endPose/{self.args.arm_end_pose_names[j]}'][i])
                    for j in range(len(self.args.localization_pose_names)):
                        self.publish_localization_pose(j, root[f'/localization/pose/{self.args.localization_pose_names[j]}'][i])
                    for j in range(len(self.args.gripper_encoder_names)):
                        self.publish_gripper_encoder(j, root[f'/gripper/encoderAngle/{self.args.gripper_encoder_names[j]}'][i], root[f'/gripper/encoderDistance/{self.args.gripper_encoder_names[j]}'][i])
                    for j in range(len(self.args.imu_9axis_names)):
                        self.publish_imu_9axis(j, root[f'/imu/9axisOrientation/{self.args.imu_9axis_names[j]}'][i], root[f'/imu/9axisAngularVelocity/{self.args.imu_9axis_names[j]}'][i], root[f'/imu/9axisLinearAcceleration/{self.args.imu_9axis_names[j]}'][i])
                    for j in range(len(self.args.lidar_point_cloud_names)):
                        if f'/lidar/pointCloud/{self.args.lidar_point_cloud_names[j]}' in root.keys():
                            if root[f'/lidar/pointCloud/{self.args.lidar_point_cloud_names[j]}'].ndim == 1 and root[f'/lidar/pointCloud/{self.args.lidar_point_cloud_names[j]}'][i].decode('utf-8')[-3:] == 'pcd':
                                self.publish_lidar_point_cloud(j, pcl.load_XYZI(os.path.join(episode_dir, root[f'/lidar/pointCloud/{self.args.lidar_point_cloud_names[j]}'][i].decode('utf-8'))).to_array())
                            else:
                                if root[f'/lidar/pointCloud/{self.args.lidar_point_cloud_names[j]}'].ndim == 1 and root[f'/lidar/pointCloud/{self.args.lidar_point_cloud_names[j]}'][i].decode('utf-8')[-3:] == 'npy':
                                    pc = np.load(os.path.join(episode_dir, root[f'/lidar/pointCloud/{self.args.lidar_point_cloud_names[j]}'][i].decode('utf-8')))
                                else:
                                    pc = root[f'/lidar/pointCloud/{self.args.lidar_point_cloud_names[j]}'][i]
                                # points = np.zeros(rgb.shape[0], dtype=dtype)
                                # points['x'] = pc[:, 0]
                                # points['y'] = pc[:, 1]
                                # points['z'] = pc[:, 2]
                                packed_points = []
                                for point in pc:
                                    x, y, z = point[:3]
                                    packed_points.append([x, y, z])
                                self.publish_lidar_point_cloud(j, packed_points)
                    for j in range(len(self.args.robot_base_vel_names)):
                        self.publish_robot_base_vel(j, root[f'/robotBase/vel/{self.args.robot_base_vel_names[j]}'][i])
                for j in range(len(self.args.lift_motor_names)):
                    self.ros_operator.publish_lift_motor(j, root[f'/lift/motor/{self.args.lift_motor_names[j]}'][i])
                print("frame:", i)
                self.rate.sleep()


def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--datasetDir', action='store', type=str, help='datasetDir.',
                        default="./data", required=False)
    parser.add_argument('--episodeName', action='store', type=str, help='Episode name.',
                        default="", required=False)
    parser.add_argument('--publishIndex', action='store', type=int, help='publishIndex',
                        default=-1, required=False)
    parser.add_argument('--type', action='store', type=str, help='type',
                        default="aloha", required=False)

    parser.add_argument('--camera_color_names', action='store', type=str, help='camera_color_names',
                        default=[],
                        required=False)
    parser.add_argument('--camera_color_topics', action='store', type=str, help='camera_color_topics',
                        default=[],
                        required=False)
    parser.add_argument('--camera_depth_names', action='store', type=str, help='camera_depth_names',
                        default=[],
                        required=False)
    parser.add_argument('--camera_depth_topics', action='store', type=str, help='camera_depth_topics',
                        default=[],
                        required=False)
    parser.add_argument('--camera_point_cloud_names', action='store', type=str, help='camera_point_cloud_names',
                        default=[],
                        required=False)
    parser.add_argument('--camera_point_cloud_topics', action='store', type=str, help='camera_point_cloud_topics',
                        default=[],
                        required=False)
    parser.add_argument('--arm_joint_state_names', action='store', type=str, help='arm_joint_state_names',
                        default=[],
                        required=False)
    parser.add_argument('--arm_joint_state_topics', action='store', type=str, help='arm_joint_state_topics',
                        default=[],
                        required=False)
    parser.add_argument('--arm_end_pose_names', action='store', type=str, help='arm_end_pose_names',
                        default=[],
                        required=False)
    parser.add_argument('--arm_end_pose_topics', action='store', type=str, help='arm_end_pose_topics',
                        default=[],
                        required=False)
    parser.add_argument('--arm_end_pose_orients', action='store', type=str, help='arm_end_pose_orients',
                        default=[],
                        required=False)
    parser.add_argument('--localization_pose_names', action='store', type=str, help='localization_pose_names',
                        default=[],
                        required=False)
    parser.add_argument('--localization_pose_topics', action='store', type=str, help='localization_pose_topics',
                        default=[],
                        required=False)
    parser.add_argument('--gripper_encoder_names', action='store', type=str, help='gripper_encoder_names',
                        default=[],
                        required=False)
    parser.add_argument('--gripper_encoder_topics', action='store', type=str, help='gripper_encoder_topics',
                        default=[],
                        required=False)
    parser.add_argument('--imu_9axis_names', action='store', type=str, help='imu_9axis_names',
                        default=[],
                        required=False)
    parser.add_argument('--imu_9axis_topics', action='store', type=str, help='imu_9axis_topics',
                        default=[],
                        required=False)
    parser.add_argument('--lidar_point_cloud_names', action='store', type=str, help='lidar_point_cloud_names',
                        default=[],
                        required=False)
    parser.add_argument('--lidar_point_cloud_topics', action='store', type=str, help='lidar_point_cloud_topics',
                        default=[],
                        required=False)
    parser.add_argument('--robot_base_vel_names', action='store', type=str, help='robot_base_vel_names',
                        default=[],
                        required=False)
    parser.add_argument('--robot_base_vel_topics', action='store', type=str, help='robot_base_vel_topics',
                        default=[],
                        required=False)
    parser.add_argument('--lift_motor_names', action='store', type=str, help='lift_motor_names',
                        default=[],
                        required=False)
    parser.add_argument('--lift_motor_topics', action='store', type=str, help='lift_motor_topics',
                        default=[],
                        required=False)
    parser.add_argument('--publish_rate', action='store', type=int, help='publish_rate',
                        default=30, required=False)
    args = parser.parse_args()

    with open(f'../install/data_tools/share/data_tools/config/{args.type}_data_params.yaml', 'r') as file:
        yaml_data = yaml.safe_load(file)
        args.camera_color_names = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('camera', {}).get('color', {}).get('names', [])
        args.camera_color_topics = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('camera', {}).get('color', {}).get('topics', [])
        args.camera_depth_names = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('camera', {}).get('depth', {}).get('names', [])
        args.camera_depth_topics = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('camera', {}).get('depth', {}).get('topics', [])
        args.camera_point_cloud_names = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('camera', {}).get('pointCloud', {}).get('names', [])
        args.camera_point_cloud_topics = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('camera', {}).get('pointCloud', {}).get('topics', [])
        args.arm_joint_state_names = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('arm', {}).get('jointState', {}).get('names', [])
        args.arm_joint_state_topics = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('arm', {}).get('jointState', {}).get('topics', [])
        args.arm_end_pose_names = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('arm', {}).get('endPose', {}).get('names', [])
        args.arm_end_pose_topics = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('arm', {}).get('endPose', {}).get('topics', [])
        args.arm_end_pose_orients = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('arm', {}).get('endPose', {}).get('orients', [])
        args.localization_pose_names = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('localization', {}).get('pose', {}).get('names', [])
        args.localization_pose_topics = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('localization', {}).get('pose', {}).get('topics', [])
        args.gripper_encoder_names = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('gripper', {}).get('encoder', {}).get('names', [])
        args.gripper_encoder_topics = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('gripper', {}).get('encoder', {}).get('topics', [])
        args.imu_9axis_names = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('imu', {}).get('9axis', {}).get('names', [])
        args.imu_9axis_topics = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('imu', {}).get('9axis', {}).get('topics', [])
        args.lidar_point_cloud_names = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('lidar', {}).get('pointCloud', {}).get('names', [])
        args.lidar_point_cloud_topics = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('lidar', {}).get('pointCloud', {}).get('topics', [])
        args.robot_base_vel_names = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('robotBase', {}).get('vel', {}).get('names', [])
        args.robot_base_vel_topics = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('robotBase', {}).get('vel', {}).get('topics', [])
        args.lift_motor_names = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('lift', {}).get('motor', {}).get('names', [])
        args.lift_motor_topics = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('lift', {}).get('motor', {}).get('topics', [])
    return args


def main():
    args = get_arguments()
    rclpy.init()
    ros_operator = RosOperator(args)
    # ros_operator.process_data()
    rclpy.spin(ros_operator)


if __name__ == '__main__':
    main()
