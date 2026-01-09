#!/usr/bin/env python3
# -- coding: UTF-8

import array
import os
import sys

import numpy as np
import h5py
import argparse
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
import rclpy.time
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField, Imu
from data_msgs.msg import Gripper
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import pcl
import threading
# import ros2_numpy
from sensor_msgs_py import point_cloud2

from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rclpy.serialization import serialize_message

import json

USELIFT = False
if USELIFT:
    from bt_task_msgs.msg import LiftMotorMsg
    from bt_task_msgs.srv import LiftMotorSrv

USECOMPRESS = True
USEDEPTH = True
USEPOINTS = False

HOME=""

def run_shell_command(cmd="", enable_error_output=True):
    if not enable_error_output:
        cmd += " 2> /dev/null"
    return os.popen(cmd).read()

class RosOperator(Node):
    def __init__(self, args): 
        super().__init__('data_publish')
        self.args = args
        self.batch = []
        self.topic_types = {}
        self.camera_color_names = self.args.camera_color_names
        self.camera_color_topics = self.args.camera_color_topics
        self.camera_color_info_topics = self.args.camera_color_info_topics
        self.camera_color_info_msgs = []
        self.camera_depth_names = self.args.camera_depth_names
        self.camera_depth_topics = self.args.camera_depth_topics
        self.camera_depth_info_topics = self.args.camera_depth_info_topics
        self.camera_depth_info_msgs = []
        self.camera_point_cloud_topics = self.args.camera_point_cloud_topics
        self.arm_joint_state_topics = self.args.arm_joint_state_topics
        self.arm_end_pose_topics = self.args.arm_end_pose_topics
        self.localization_pose_topics = self.args.localization_pose_topics
        self.gripper_encoder_topics = self.args.gripper_encoder_topics
        self.imu_9axis_topics = self.args.imu_9axis_topics
        self.lidar_point_cloud_topics = self.args.lidar_point_cloud_topics
        self.robot_base_vel_topics = self.args.robot_base_vel_topics
        self.lift_motor_topics = self.args.lift_motor_topics
        self.bridge = CvBridge()
        self.init_mcap()
        self.init_ros()
        self.thread = threading.Thread(target=self.process_data)
        self.thread.start()

    def init_mcap(self):                
        episode_dir = "episode" + str(self.args.episodeIndex)
        mcap_dir = os.path.join(HOME, 'cattleH/white/mcap', episode_dir)
        # episode_dir = os.path.join(self.args.datasetDir, "episode" + str(self.args.episodeIndex))
        # mcap_dir = os.path.join(HOME, 'mcap', os.path.relpath(episode_dir, HOME))
        print(mcap_dir)
        if os.path.exists(mcap_dir):
            print(run_shell_command(f"rm -r {mcap_dir}"))
        
        storage_options_output = StorageOptions(uri=mcap_dir, storage_id="mcap")
        self.mcap_writer = SequentialWriter()
        try:
            self.mcap_writer.open(storage_options_output, ConverterOptions("", ""))
        except RuntimeError:
            print(f"[error] can't wirte data to {mcap_dir}")
            return

    def init_ros(self):
        camera_type_name = "sensor_msgs/msg/Image"
        if USECOMPRESS:
            camera_type_name = "sensor_msgs/msg/CompressedImage"
        self.bind_topic_and_type_from_list(self.camera_color_topics, camera_type_name)
        self.bind_topic_and_type_from_list(self.camera_color_info_topics, "sensor_msgs/msg/CameraInfo")
        if USEDEPTH:
            self.bind_topic_and_type_from_list(self.camera_depth_topics, camera_type_name)
            self.bind_topic_and_type_from_list(self.camera_depth_info_topics, "sensor_msgs/msg/CameraInfo")
        if USEPOINTS:
            self.bind_topic_and_type_from_list(self.camera_point_cloud_topics, "sensor_msgs/msg/PointCloud2")
        self.bind_topic_and_type_from_list(self.arm_joint_state_topics, "sensor_msgs/msg/JointState")
        self.bind_topic_and_type_from_list(self.arm_end_pose_topics, "geometry_msgs/msg/PoseStamped")
        self.bind_topic_and_type_from_list(self.localization_pose_topics, "geometry_msgs/msg/PoseStamped")
        self.bind_topic_and_type_from_list(self.gripper_encoder_topics, "data_msgs/msg/Gripper")
        self.bind_topic_and_type_from_list(self.imu_9axis_topics, "sensor_msgs/msg/Imu")
        self.bind_topic_and_type_from_list(self.lidar_point_cloud_topics, "sensor_msgs/msg/PointCloud2")
        self.bind_topic_and_type_from_list(self.robot_base_vel_topics, "geometry_msgs/msg/Twist")

        if USELIFT:
            pass
            # self.lift_motor_publishers = [self.create_publisher(LiftMotorMsg, topic, 10) for topic in self.lift_motor_topics]
            # self.bind_topic_and_type_from_list(self.lift_motor_topics, "bt_task_msgs/msg/LiftMotorMsg")
        
        for key,value in self.topic_types.items():
            print(f"{key}: {value}")


    def bind_topic_and_type_from_list(self, topic_list : list, topic_type):
        for topic_name in topic_list:
            self.topic_types[topic_name] = topic_type
            self.create_topic_to_mcap(topic_name, topic_type)

    def create_topic_to_mcap(self, topic_name, topic_type):
        topic_metadata = TopicMetadata(
            topic_name, topic_type, 'cdr', 
        )
        self.mcap_writer.create_topic(topic_metadata)

    def get_topic_typename(self, topic_name):
        return self.topic_types[topic_name]

    def write_data_to_mcap(self):
        for b_topic, b_data, b_timestamp in self.batch:
            self.mcap_writer.write(b_topic, b_data, b_timestamp)
        self.batch = []

    def push_data_to_mcap(self, data, topic_name):
        self.batch.append((topic_name, serialize_message(data), self.frame_ros_time.nanoseconds))

        if len(self.batch) >= 2000:
            self.write_data_to_mcap()

    def publish_camera_color(self, index, color):
        if USECOMPRESS:
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = self.frame_ros_time.to_msg()
            compressed_msg.format = "jpeg"
            compressed_msg.data = array.array("B", color)
            self.push_data_to_mcap(compressed_msg, self.camera_color_topics[index])
        else:
            msg = self.bridge.cv2_to_imgmsg(color, "bgr8")
            msg.header.stamp = self.frame_ros_time.to_msg()
            self.push_data_to_mcap(msg, self.camera_color_topics[index])
        local_color_info_msg = self.camera_color_info_msgs[index]
        local_color_info_msg.header.stamp = self.frame_ros_time.to_msg()
        self.push_data_to_mcap(local_color_info_msg, self.camera_color_info_topics[index])

    def publish_camera_depth(self, index, depth):
        if USECOMPRESS:
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = self.frame_ros_time.to_msg()
            compressed_msg.format = "png"
            compressed_msg.data = array.array("B", depth)
            self.push_data_to_mcap(compressed_msg, self.camera_depth_topics[index])
        else:
            msg = self.bridge.cv2_to_imgmsg(depth, "16UC1")
            msg.header.stamp = self.frame_ros_time.to_msg()
            self.push_data_to_mcap(msg, self.camera_depth_topics[index])
        local_depth_info_msg = self.camera_depth_info_msgs[index]
        local_depth_info_msg.header.stamp = self.frame_ros_time.to_msg()
        self.push_data_to_mcap(local_depth_info_msg, self.camera_depth_info_topics[index])

    def publish_camera_point_cloud(self, index, point_cloud):
        data = self.pcd_to_msg_rgb(point_cloud)
        data.header.stamp = self.frame_ros_time.to_msg()
        self.push_data_to_mcap(data, self.camera_point_cloud_topics[index])

    def publish_arm_joint_state(self, index, joint_state):
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.frame_ros_time.to_msg()
        joint_state_msg.name = [f'joint{i}' for i in range(len(joint_state))]
        joint_state_msg.position = array.array("d", joint_state.tolist())
        joint_state_msg.effort = array.array("d", [0, 0, 0, 0, 0, 0, 0])
        joint_state_msg.velocity = array.array("d", [0, 0, 0, 0, 0, 0, 0])
        self.push_data_to_mcap(joint_state_msg, self.arm_joint_state_topics[index])

    def publish_arm_end_pose(self, index, end_pose):
        end_pose_msg = PoseStamped()
        end_pose_msg.header = Header()
        end_pose_msg.header.frame_id = "map"
        end_pose_msg.header.stamp = self.frame_ros_time.to_msg()
        if self.arm_end_pose_orients[index]:
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
        self.push_data_to_mcap(end_pose_msg, self.arm_end_pose_topics[index])

    def publish_localization_pose(self, index, pose):
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = self.frame_ros_time.to_msg()
        pose_msg.pose.position.x = pose[0]
        pose_msg.pose.position.y = pose[1]
        pose_msg.pose.position.z = pose[2]
        q = quaternion_from_euler(pose[3], pose[4], pose[5])
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        self.push_data_to_mcap(pose_msg, self.localization_pose_topics[index])

    def publish_gripper_encoder(self, index, encoder_angle, encoder_distance):
        gripper_msg = Gripper()
        gripper_msg.header = Header()
        gripper_msg.header.frame_id = "map"
        gripper_msg.header.stamp = self.frame_ros_time.to_msg()
        gripper_msg.angle = encoder_angle
        gripper_msg.distance = encoder_distance
        self.push_data_to_mcap(gripper_msg, self.gripper_encoder_topics[index])

    def publish_imu_9axis(self, index, orientation, angular_velocity, linear_acceleration):
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.frame_id = "map"
        imu_msg.header.stamp = self.frame_ros_time.to_msg()
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
        self.push_data_to_mcap(imu_msg, self.imu_9axis_topics[index])

    def publish_lidar_point_cloud(self, index, point_cloud):
        data = self.pcd_to_msg(point_cloud)
        data.header.stamp = self.frame_ros_time.to_msg()
        self.push_data_to_mcap(data, self.lidar_point_cloud_topics[index])

    def publish_robot_base_vel(self, index, vel):
        vel_msg = Twist()
        vel_msg.linear.x = vel[0]
        vel_msg.linear.y = vel[1]
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = vel[2]
        self.push_data_to_mcap(vel_msg, self.robot_base_vel_topics[index])

    def publish_lift_motor(self, index, val):
        if USELIFT:
            # rospy.wait_for_service(self.lift_motor_topics[index])
            try:
                client = self.create_client(LiftMotorSrv, self.lift_motor_topics[index])
                req = LiftMotorSrv.Request()
                req.val = val
                req.mode = 0
                response = client.call_async(req)
                return response
            except rclpy.ServiceException as e:
                rclpy.logerr("Service call failed: %s" % e)
                return None

    def pcd_to_msg_rgb(self, points):
        header = Header(frame_id="camera", stamp=self.frame_ros_time.to_msg())
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
        ]
        pointcloud_msg = point_cloud2.create_cloud(header, fields, points)
        return pointcloud_msg

    def pcd_to_msg(self, points):
        header = Header(frame_id="lidar", stamp=self.frame_ros_time.to_msg())
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        pointcloud_msg = point_cloud2.create_cloud(header, fields, points)
        return pointcloud_msg

    def get_camera_info_msg(self, info_path) -> CameraInfo:
        print(info_path)
        info_msg = CameraInfo()
        with open(info_path, 'r') as info_file:
            data = json.load(info_file)
            info_msg.d = array.array('d', data["D"])
            info_msg.k = np.array(data["K"])
            info_msg.p = np.array(data["P"])
            info_msg.r = np.array(data["R"], dtype=np.float64)
            info_msg.binning_x = data["binning_x"]
            info_msg.binning_y = data["binning_y"]
            info_msg.distortion_model = data["distortion_model"]
            info_msg.height = data["height"]
            info_msg.width = data["width"]
        return info_msg

    def process_data(self):
        episode_dir = os.path.join(self.args.datasetDir, "episode" + str(self.args.episodeIndex))
        data_path = os.path.join(episode_dir, 'data.hdf5')
        self.rate = self.create_rate(self.args.publish_rate)
        time_dt = 1 / self.args.publish_rate
        with h5py.File(data_path, 'r') as root:
            max_action_len = root['size'][()]
            self.frame_ros_time = self.get_clock().now()
            for j in range(len(self.camera_color_names)):
                self.camera_color_info_msgs.append(self.get_camera_info_msg(os.path.join(episode_dir, f"camera/color/{self.camera_color_names[j]}/config.json")))
            for j in range(len(self.camera_depth_names)):
                self.camera_depth_info_msgs.append(self.get_camera_info_msg(os.path.join(episode_dir, f"camera/depth/{self.camera_depth_names[j]}/config.json")))
            for i in range(max_action_len):
                sys.stdout.write('\r[%3d%%] frame progress ( %d / %d )' % (int((i+1)/max_action_len*100), i+1, max_action_len))
                sys.stdout.flush()
                if not rclpy.ok():
                    return
                # self.frame_ros_time += rclpy.duration.Duration(seconds=time_dt)

                for j in range(len(self.camera_color_names)):
                    # 750us + 500us + (200~250)us    [1500us]
                    # 200us + 500us + (60~100)us     [800us]
                    # 250us + (80~120)us    [350us]
                    if root[f'/camera/color/{self.camera_color_names[j]}'].ndim == 1:
                        jpg_path = os.path.join(episode_dir, root[f'/camera/color/{self.camera_color_names[j]}'][i].decode('utf-8'))
                        
                        if j == 0:
                            base_name, extension = os.path.splitext(os.path.basename(jpg_path))
                            self.frame_ros_time = rclpy.time.Time(seconds=float(base_name))
                        if USECOMPRESS:
                            with open(jpg_path, 'rb') as image_file:
                                image_data = image_file.read()
                                self.publish_camera_color(j, image_data)
                        else:
                            self.publish_camera_color(j, cv2.imread(os.path.join(episode_dir, root[f'/camera/color/{self.camera_color_names[j]}'][i].decode('utf-8')), cv2.IMREAD_UNCHANGED))
                    else:
                        self.publish_camera_color(j, root[f'/camera/color/{self.camera_color_names[j]}'][i])
                if USEDEPTH:
                    for j in range(len(self.camera_depth_names)):
                        if root[f'/camera/depth/{self.camera_depth_names[j]}'].ndim == 1:
                            if USECOMPRESS:
                                with open(os.path.join(episode_dir, root[f'/camera/depth/{self.camera_depth_names[j]}'][i].decode('utf-8')), 'rb') as depth_file:
                                    depth_data = depth_file.read()
                                    self.publish_camera_depth(j, depth_data)
                            else:
                                self.publish_camera_depth(j, cv2.imread(os.path.join(episode_dir, root[f'/camera/depth/{self.camera_depth_names[j]}'][i].decode('utf-8')), cv2.IMREAD_UNCHANGED))
                        else:
                            self.publish_camera_depth(j, root[f'/camera/depth/{self.camera_depth_names[j]}'][i])
                if USEPOINTS:
                    for j in range(len(self.args.camera_point_cloud_names)):
                        if f'/lidar/pointCloud/{self.args.camera_point_cloud_names[j]}' in root.keys():
                            if root[f'/camera/pointCloud/{self.args.camera_point_cloud_names[j]}'].ndim == 1 and root[f'/camera/pointCloud/{self.args.camera_point_cloud_names[j]}'][i].decode('utf-8')[-3:] == 'pcd':
                                self.publish_camera_point_cloud(j, pcl.load_XYZRGB(os.path.join(episode_dir, root[f'/camera/pointCloud/{self.args.camera_point_cloud_names[j]}'][i].decode('utf-8'))).to_array())
                            else:
                                if root[f'/camera/pointCloud/{self.args.camera_point_cloud_names[j]}'].ndim == 1 and root[f'/camera/pointCloud/{self.args.camera_point_cloud_names[j]}'][i].decode('utf-8')[-3:] == 'npy':
                                    pc = np.load(os.path.join(episode_dir, root[f'/camera/pointCloud/{self.args.camera_point_cloud_names[j]}'][i].decode('utf-8')))
                                else:
                                    pc = root[f'/camera/pointCloud/{self.args.camera_point_cloud_names[j]}'][i]
                                packed_points = []
                                for point in pc:
                                    x, y, z = point[:3]
                                    r, g, b = point[3:]
                                    rgb = (int(r) << 16) | (int(g) << 8) | int(b)
                                    packed_points.append([x, y, z, rgb])
                                self.publish_camera_point_cloud(j, packed_points)
                for j in range(len(self.args.arm_joint_state_names)):
                    # 140us + (90)us   [250us]
                    # print("---")
                    joint_data = root[f'/arm/jointStatePosition/{self.args.arm_joint_state_names[j]}'][i]
                    self.publish_arm_joint_state(j, joint_data)
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
            self.write_data_to_mcap()
            rclpy.shutdown()

def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--datasetDir', action='store', type=str, help='datasetDir.',
                        default="./data", required=False)
    parser.add_argument('--episodeIndex', action='store', type=int, help='Episode index.',
                        default=0, required=False)
    parser.add_argument('--publishIndex', action='store', type=int, help='publishIndex',
                        default=-1, required=False)

    parser.add_argument('--camera_color_names', action='store', type=str, help='camera_color_names',
                        default=['left', 'front', 'right'],
                        required=False)
    parser.add_argument('--camera_color_topics', action='store', type=str, help='camera_color_topics',
                        default=['/camera_l/color/image_raw', '/camera_f/color/image_raw', '/camera_r/color/image_raw'],
                        required=False)
    parser.add_argument('--camera_color_info_topics', action='store', type=str, help='camera_color_topics',
                        default=['/camera_l/color/camera_info', '/camera_f/color/camera_info', '/camera_r/color/camera_info'],
                        required=False)
    parser.add_argument('--camera_depth_names', action='store', type=str, help='camera_depth_names',
                        default=['left', 'front', 'right'],
                        required=False)
    parser.add_argument('--camera_depth_topics', action='store', type=str, help='camera_depth_topics',
                        default=['/camera_l/aligned_depth_to_color/image_raw', '/camera_f/aligned_depth_to_color/image_raw', '/camera_r/aligned_depth_to_color/image_raw'],
                        required=False)
    parser.add_argument('--camera_depth_info_topics', action='store', type=str, help='camera_depth_topics',
                        default=['/camera_l/aligned_depth_to_color/camera_info', '/camera_f/aligned_depth_to_color/camera_info', '/camera_r/aligned_depth_to_color/camera_info'],
                        required=False)
    parser.add_argument('--camera_point_cloud_names', action='store', type=str, help='camera_point_cloud_names',
                        default=['left', 'front', 'right'],
                        required=False)
    parser.add_argument('--camera_point_cloud_topics', action='store', type=str, help='camera_point_cloud_topics',
                        default=['/camera_l/depth/color/points', '/camera_f/depth/color/points', '/camera_r/depth/color/points'],
                        required=False)
    parser.add_argument('--arm_joint_state_names', action='store', type=str, help='arm_joint_state_names',
                        default=['masterLeft', 'masterRight', 'puppetLeft', 'puppetRight'],
                        required=False)
    parser.add_argument('--arm_joint_state_topics', action='store', type=str, help='arm_joint_state_topics',
                        default=['/master/joint_left', '/master/joint_right', '/puppet/joint_left', '/puppet/joint_right'],
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
                        default=['puppetLeft', 'puppetRight'],
                        required=False)
    parser.add_argument('--localization_pose_topics', action='store', type=str, help='localization_pose_topics',
                        default=['/puppet/end_pose_left', '/puppet/end_pose_right'],
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
    return args


def main():
    global HOME
    if "HOME" in os.environ:
        HOME = os.environ["HOME"]
    else:
        HOME = "/home/agilex"

    args = get_arguments()
    rclpy.init()
    ros_operator = RosOperator(args)

    try:
        rclpy.spin(ros_operator)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
