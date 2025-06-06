#!/home/lin/miniconda3/envs/aloha/bin/python
# -- coding: UTF-8
"""
#!/root/miniconda3/envs/aloha/bin/python
#!/home/lin/miniconda3/envs/aloha/bin/python
"""

import os
import time

import cv2
import numpy as np
import h5py
import argparse
import dm_env

import collections
from collections import deque

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField, Imu
from data_msgs.msg import Gripper
from sensor_msgs import point_cloud2
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import quaternion_from_euler
import pcl
import ros_numpy
import yaml
USELIFT = False
if USELIFT:
    from bt_task_msgs.msg import LiftMotorMsg
    from bt_task_msgs.srv import LiftMotorSrv, LiftMotorSrvRequest, LiftMotorSrvResponse


def pcd_to_msg(points):
    # point_fields = [PointField(name='x', offset=0,
    #                            datatype=PointField.FLOAT32, count=1),
    #                 PointField(name='y', offset=4,
    #                            datatype=PointField.FLOAT32, count=1),
    #                 PointField(name='z', offset=8,
    #                            datatype=PointField.FLOAT32, count=1),
    #                 PointField(name='rgb', offset=12,
    #                            datatype=PointField.UINT32, count=1)
    #                 ]
    # header = Header(frame_id="camera", stamp=rospy.Time.now())
    # points_byte = points[:, 0:4]#.tobytes()
    # return PointCloud2(header=header,
    #                    height=1,
    #                    width=len(points),
    #                    is_dense=False,
    #                    is_bigendian=True,
    #                    fields=point_fields,
    #                    point_step=int(len(points_byte) / len(points)),
    #                    row_step=len(points_byte),
    #                    data=points_byte)
    return ros_numpy.point_cloud2.array_to_pointcloud2(points, stamp=rospy.Time.now(), frame_id="camera")


# 保存数据函数
def process_data(args, ros_operator):

    episode_dir = os.path.join(args.datasetDir, "episode" + str(args.episodeIndex))
    data_path = os.path.join(episode_dir, 'data.hdf5')
    if not os.path.exists(data_path):
        data_path = os.path.join(args.datasetDir, "episode" + str(args.episodeIndex) + '.hdf5')
    rate = rospy.Rate(args.publish_rate)
    with h5py.File(data_path, 'r') as root:
        max_action_len = root['size'][()]
        if args.publishIndex != -1:
            while not rospy.is_shutdown():
                i = args.publishIndex
                for j in range(len(args.camera_color_names)):
                    if root[f'/camera/color/{args.camera_color_names[j]}'].ndim == 1:
                        ros_operator.publish_camera_color(j, cv2.imread(os.path.join(episode_dir, root[f'/camera/color/{args.camera_color_names[j]}'][i].decode('utf-8')), cv2.IMREAD_UNCHANGED))
                    else:
                        ros_operator.publish_camera_color(j, root[f'/camera/color/{args.camera_color_names[j]}'][i])
                for j in range(len(args.camera_depth_names)):
                    if root[f'/camera/depth/{args.camera_depth_names[j]}'].ndim == 1:
                        ros_operator.publish_camera_depth(j, cv2.imread(os.path.join(episode_dir, root[f'/camera/depth/{args.camera_depth_names[j]}'][i].decode('utf-8')), cv2.IMREAD_UNCHANGED))
                    else:
                        ros_operator.publish_camera_depth(j, root[f'/camera/depth/{args.camera_depth_names[j]}'][i])
                for j in range(len(args.camera_point_cloud_names)):
                    if f'/lidar/pointCloud/{args.camera_point_cloud_names[j]}' in root.keys():
                        if root[f'/camera/pointCloud/{args.camera_point_cloud_names[j]}'].ndim == 1 and root[f'/camera/pointCloud/{args.camera_point_cloud_names[j]}'][i].decode('utf-8')[-3:] == 'pcd':
                            ros_operator.publish_camera_point_cloud(j, pcl.load_XYZRGB(os.path.join(episode_dir, root[f'/camera/pointCloud/{args.camera_point_cloud_names[j]}'][i].decode('utf-8'))).to_array())
                        else:
                            if root[f'/camera/pointCloud/{args.camera_point_cloud_names[j]}'].ndim == 1 and root[f'/camera/pointCloud/{args.camera_point_cloud_names[j]}'][i].decode('utf-8')[-3:] == 'npy':
                                pc = np.load(os.path.join(episode_dir, root[f'/camera/pointCloud/{args.camera_point_cloud_names[j]}'][i].decode('utf-8')))
                            else:
                                pc = root[f'/camera/pointCloud/{args.camera_point_cloud_names[j]}'][i]
                            rgb = (pc[:, 3]).astype(np.uint32)*(2**16) + (pc[:, 4]).astype(np.uint32)*(2**8) + (pc[:, 5]).astype(np.uint32)
                            dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.uint32)]
                            points = np.zeros(rgb.shape[0], dtype=dtype)
                            points['x'] = pc[:, 0]
                            points['y'] = pc[:, 1]
                            points['z'] = pc[:, 2]
                            points['rgb'] = rgb
                            # pc[:, 3] = rgb
                            # rgb = rgb[:, np.newaxis]
                            # pc = np.concatenate([pc[:, :3], rgb], axis=1).astype(np.float32)
                            # print(pc)
                            ros_operator.publish_camera_point_cloud(j, points)
                for j in range(len(args.arm_joint_state_names)):
                    ros_operator.publish_arm_joint_state(j, root[f'/arm/jointStatePosition/{args.arm_joint_state_names[j]}'][i])
                for j in range(len(args.arm_end_pose_names)):
                    ros_operator.publish_arm_end_pose(j, root[f'/arm/endPose/{args.arm_end_pose_names[j]}'][i])
                for j in range(len(args.localization_pose_names)):
                    ros_operator.publish_localization_pose(j, root[f'/localization/pose/{args.localization_pose_names[j]}'][i])
                for j in range(len(args.gripper_encoder_names)):
                    ros_operator.publish_gripper_encoder(j, root[f'/gripper/encoderAngle/{args.gripper_encoder_names[j]}'][i], root[f'/gripper/encoderDistance/{args.gripper_encoder_names[j]}'][i])
                for j in range(len(args.imu_9axis_names)):
                    ros_operator.publish_imu_9axis(j, root[f'/imu/9axisOrientation/{args.imu_9axis_names[j]}'][i], root[f'/imu/9axisAngularVelocity/{args.imu_9axis_names[j]}'][i], root[f'/imu/9axisLinearAcceleration/{args.imu_9axis_names[j]}'][i])
                for j in range(len(args.lidar_point_cloud_names)):
                    if f'/lidar/pointCloud/{args.lidar_point_cloud_names[j]}' in root.keys():
                        if root[f'/lidar/pointCloud/{args.lidar_point_cloud_names[j]}'].ndim == 1 and root[f'/lidar/pointCloud/{args.lidar_point_cloud_names[j]}'][i].decode('utf-8')[-3:] == 'pcd':
                            ros_operator.publish_lidar_point_cloud(j, pcl.load_XYZI(os.path.join(episode_dir, root[f'/lidar/pointCloud/{args.lidar_point_cloud_names[j]}'][i].decode('utf-8'))).to_array())
                        else:
                            if root[f'/lidar/pointCloud/{args.lidar_point_cloud_names[j]}'].ndim == 1 and root[f'/lidar/pointCloud/{args.lidar_point_cloud_names[j]}'][i].decode('utf-8')[-3:] == 'npy':
                                pc = np.load(os.path.join(episode_dir, root[f'/lidar/pointCloud/{args.lidar_point_cloud_names[j]}'][i].decode('utf-8')))
                            else:
                                pc = root[f'/lidar/pointCloud/{args.lidar_point_cloud_names[j]}'][i]
                            points = np.zeros(rgb.shape[0], dtype=dtype)
                            points['x'] = pc[:, 0]
                            points['y'] = pc[:, 1]
                            points['z'] = pc[:, 2]
                            ros_operator.publish_lidar_point_cloud(j, points)
                for j in range(len(args.robot_base_vel_names)):
                    ros_operator.publish_robot_base_vel(j, root[f'/robotBase/vel/{args.robot_base_vel_names[j]}'][i])
                for j in range(len(args.lift_motor_names)):
                    ros_operator.publish_lift_motor(j, root[f'/lift/motor/{args.lift_motor_names[j]}'][i])
                print("frame:", i)
                rate.sleep()
        else:
            for i in range(max_action_len):
                if rospy.is_shutdown():
                    return
                for j in range(len(args.camera_color_names)):
                    if root[f'/camera/color/{args.camera_color_names[j]}'].ndim == 1:
                        ros_operator.publish_camera_color(j, cv2.imread(os.path.join(episode_dir, root[f'/camera/color/{args.camera_color_names[j]}'][i].decode('utf-8')), cv2.IMREAD_UNCHANGED))
                    else:
                        ros_operator.publish_camera_color(j, root[f'/camera/color/{args.camera_color_names[j]}'][i])
                for j in range(len(args.camera_depth_names)):
                    if root[f'/camera/depth/{args.camera_depth_names[j]}'].ndim == 1:
                        ros_operator.publish_camera_depth(j, cv2.imread(os.path.join(episode_dir, root[f'/camera/depth/{args.camera_depth_names[j]}'][i].decode('utf-8')), cv2.IMREAD_UNCHANGED))
                    else:
                        ros_operator.publish_camera_depth(j, root[f'/camera/depth/{args.camera_depth_names[j]}'][i])
                for j in range(len(args.camera_point_cloud_names)):
                    if f'/lidar/pointCloud/{args.camera_point_cloud_names[j]}' in root.keys():
                        if root[f'/camera/pointCloud/{args.camera_point_cloud_names[j]}'].ndim == 1 and root[f'/camera/pointCloud/{args.camera_point_cloud_names[j]}'][i].decode('utf-8')[-3:] == 'pcd':
                            ros_operator.publish_camera_point_cloud(j, pcl.load_XYZRGB(os.path.join(episode_dir, root[f'/camera/pointCloud/{args.camera_point_cloud_names[j]}'][i].decode('utf-8'))).to_array())
                        else:
                            if root[f'/camera/pointCloud/{args.camera_point_cloud_names[j]}'].ndim == 1 and root[f'/camera/pointCloud/{args.camera_point_cloud_names[j]}'][i].decode('utf-8')[-3:] == 'npy':
                                pc = np.load(os.path.join(episode_dir, root[f'/camera/pointCloud/{args.camera_point_cloud_names[j]}'][i].decode('utf-8')))
                            else:
                                pc = root[f'/camera/pointCloud/{args.camera_point_cloud_names[j]}'][i]
                            rgb = (pc[:, 3]).astype(np.uint32)*(2**16) + (pc[:, 4]).astype(np.uint32)*(2**8) + (pc[:, 5]).astype(np.uint32)
                            dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.uint32)]
                            points = np.zeros(rgb.shape[0], dtype=dtype)
                            points['x'] = pc[:, 0]
                            points['y'] = pc[:, 1]
                            points['z'] = pc[:, 2]
                            points['rgb'] = rgb
                            # pc[:, 3] = rgb
                            # rgb = rgb[:, np.newaxis]
                            # pc = np.concatenate([pc[:, :3], rgb], axis=1).astype(np.float32)
                            # print(pc)
                            ros_operator.publish_camera_point_cloud(j, points)
                for j in range(len(args.arm_joint_state_names)):
                    ros_operator.publish_arm_joint_state(j, root[f'/arm/jointStatePosition/{args.arm_joint_state_names[j]}'][i])
                for j in range(len(args.arm_end_pose_names)):
                    ros_operator.publish_arm_end_pose(j, root[f'/arm/endPose/{args.arm_end_pose_names[j]}'][i])
                for j in range(len(args.localization_pose_names)):
                    ros_operator.publish_localization_pose(j, root[f'/localization/pose/{args.localization_pose_names[j]}'][i])
                for j in range(len(args.gripper_encoder_names)):
                    ros_operator.publish_gripper_encoder(j, root[f'/gripper/encoderAngle/{args.gripper_encoder_names[j]}'][i], root[f'/gripper/encoderDistance/{args.gripper_encoder_names[j]}'][i])
                for j in range(len(args.imu_9axis_names)):
                    ros_operator.publish_imu_9axis(j, root[f'/imu/9axisOrientation/{args.imu_9axis_names[j]}'][i], root[f'/imu/9axisAngularVelocity/{args.imu_9axis_names[j]}'][i], root[f'/imu/9axisLinearAcceleration/{args.imu_9axis_names[j]}'][i])
                for j in range(len(args.lidar_point_cloud_names)):
                    if f'/lidar/pointCloud/{args.lidar_point_cloud_names[j]}' in root.keys():
                        if root[f'/lidar/pointCloud/{args.lidar_point_cloud_names[j]}'].ndim == 1 and root[f'/lidar/pointCloud/{args.lidar_point_cloud_names[j]}'][i].decode('utf-8')[-3:] == 'pcd':
                            ros_operator.publish_lidar_point_cloud(j, pcl.load_XYZI(os.path.join(episode_dir, root[f'/lidar/pointCloud/{args.lidar_point_cloud_names[j]}'][i].decode('utf-8'))).to_array())
                        else:
                            if root[f'/lidar/pointCloud/{args.lidar_point_cloud_names[j]}'].ndim == 1 and root[f'/lidar/pointCloud/{args.lidar_point_cloud_names[j]}'][i].decode('utf-8')[-3:] == 'npy':
                                pc = np.load(os.path.join(episode_dir, root[f'/lidar/pointCloud/{args.lidar_point_cloud_names[j]}'][i].decode('utf-8')))
                            else:
                                pc = root[f'/lidar/pointCloud/{args.lidar_point_cloud_names[j]}'][i]
                            points = np.zeros(rgb.shape[0], dtype=dtype)
                            points['x'] = pc[:, 0]
                            points['y'] = pc[:, 1]
                            points['z'] = pc[:, 2]
                            ros_operator.publish_lidar_point_cloud(j, points)
                for j in range(len(args.robot_base_vel_names)):
                    ros_operator.publish_robot_base_vel(j, root[f'/robotBase/vel/{args.robot_base_vel_names[j]}'][i])
                for j in range(len(args.lift_motor_names)):
                    ros_operator.publish_lift_motor(j, root[f'/lift/motor/{args.lift_motor_names[j]}'][i])
                print("frame:", i)
                rate.sleep()


class RosOperator:
    def __init__(self, args):
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

    def init_ros(self):
        rospy.init_node('replay_episodes', anonymous=True)
        self.bridge = CvBridge()
        self.camera_color_publishers = [rospy.Publisher(topic, Image, queue_size=10) for topic in self.args.camera_color_topics]
        self.camera_depth_publishers = [rospy.Publisher(topic, Image, queue_size=10) for topic in self.args.camera_depth_topics]
        self.camera_point_cloud_publishers = [rospy.Publisher(topic, PointCloud2, queue_size=10) for topic in self.args.camera_point_cloud_topics]
        self.arm_joint_state_publishers = [rospy.Publisher(topic, JointState, queue_size=10) for topic in self.args.arm_joint_state_topics]
        self.arm_end_pose_publishers = [rospy.Publisher(topic, PoseStamped, queue_size=10) for topic in self.args.arm_end_pose_topics]
        self.localization_pose_publishers = [rospy.Publisher(topic, PoseStamped, queue_size=10) for topic in self.args.localization_pose_topics]
        self.gripper_encoder_publishers = [rospy.Publisher(topic, Gripper, queue_size=10) for topic in self.args.gripper_encoder_topics]
        self.imu_9axis_publishers = [rospy.Publisher(topic, Imu, queue_size=10) for topic in self.args.imu_9axis_topics]
        self.lidar_point_cloud_publishers = [rospy.Publisher(topic, PointCloud2, queue_size=10) for topic in self.args.lidar_point_cloud_topics]
        self.robot_base_vel_publishers = [rospy.Publisher(topic, Twist, queue_size=10) for topic in self.args.robot_base_vel_topics]
        if USELIFT:
            self.lift_motor_publishers = [rospy.Publisher(topic, LiftMotorMsg, queue_size=10) for topic in self.args.lift_motor_topics]

    def publish_camera_color(self, index, color):
        msg = self.bridge.cv2_to_imgmsg(color, "bgr8")
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        self.camera_color_publishers[index].publish(msg)

    def publish_camera_depth(self, index, depth):
        msg = self.bridge.cv2_to_imgmsg(depth, "16UC1")
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        self.camera_depth_publishers[index].publish(msg)

    def publish_camera_point_cloud(self, index, point_cloud):
        msg = pcd_to_msg(point_cloud)
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        self.camera_point_cloud_publishers[index].publish(msg)

    def publish_arm_joint_state(self, index, joint_state):
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = [f'joint{i}' for i in range(len(joint_state))]
        joint_state_msg.position = joint_state
        self.arm_joint_state_publishers[index].publish(joint_state_msg)

    def publish_arm_end_pose(self, index, end_pose):
        end_pose_msg = PoseStamped()
        end_pose_msg.header = Header()
        end_pose_msg.header.frame_id = "map"
        end_pose_msg.header.stamp = rospy.Time.now()
        end_pose_msg.pose.position.x = end_pose[0]
        end_pose_msg.pose.position.y = end_pose[1]
        end_pose_msg.pose.position.z = end_pose[2]
        if self.args.arm_end_pose_orients[index]:
            q = quaternion_from_euler(end_pose[3], end_pose[4], end_pose[5])
            end_pose_msg.pose.orientation.x = q[0]
            end_pose_msg.pose.orientation.y = q[1]
            end_pose_msg.pose.orientation.z = q[2]
            end_pose_msg.pose.orientation.w = q[3]
        else:
            end_pose_msg.pose.orientation.x = end_pose[3]
            end_pose_msg.pose.orientation.y = end_pose[4]
            end_pose_msg.pose.orientation.z = end_pose[5]
            end_pose_msg.pose.orientation.w = end_pose[6]
        self.arm_end_pose_publishers[index].publish(end_pose_msg)

    def publish_localization_pose(self, index, pose):
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = rospy.Time.now()
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
        gripper_msg.header.stamp = rospy.Time.now()
        gripper_msg.angle = encoder_angle
        gripper_msg.distance = encoder_distance
        self.gripper_encoder_publishers[index].publish(gripper_msg)

    def publish_imu_9axis(self, index, orientation, angular_velocity, linear_acceleration):
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.frame_id = "map"
        imu_msg.header.stamp = rospy.Time.now()
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
        self.lidar_point_cloud_publishers[index].publish(pcd_to_msg(point_cloud))

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
            # rospy.wait_for_service(self.args.lift_motor_topics[index])
            try:
                lift_motor_srv = rospy.ServiceProxy(self.args.lift_motor_topics[index], LiftMotorSrv)
                req = LiftMotorSrvRequest()
                req.val = val
                req.mode = 0
                response = lift_motor_srv(req)
                return response
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
                return None


def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--datasetDir', action='store', type=str, help='datasetDir.',
                        default="./data", required=False)
    parser.add_argument('--episodeIndex', action='store', type=int, help='Episode index.',
                        default=0, required=False)
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

    with open(f'../install/share/data_tools/config/{args.type}_data_params.yaml', 'r') as file:
        yaml_data = yaml.safe_load(file)
        args.camera_color_names = yaml_data['dataInfo']['camera']['color']['names']
        args.camera_color_topics = yaml_data['dataInfo']['camera']['color']['topics']
        args.camera_depth_names = yaml_data['dataInfo']['camera']['depth']['names']
        args.camera_depth_topics = yaml_data['dataInfo']['camera']['depth']['topics']
        args.camera_point_cloud_names = yaml_data['dataInfo']['camera']['pointCloud']['names']
        args.camera_point_cloud_topics = yaml_data['dataInfo']['camera']['pointCloud']['topics']
        args.arm_joint_state_names = yaml_data['dataInfo']['arm']['jointState']['names']
        args.arm_joint_state_topics = yaml_data['dataInfo']['arm']['jointState']['topics']
        args.arm_end_pose_names = yaml_data['dataInfo']['arm']['endPose']['names']
        args.arm_end_pose_topics = yaml_data['dataInfo']['arm']['endPose']['topics']
        args.arm_end_pose_orients = yaml_data['dataInfo']['arm']['endPose']['orients']
        args.localization_pose_names = yaml_data['dataInfo']['localization']['pose']['names']
        args.localization_pose_topics = yaml_data['dataInfo']['localization']['pose']['topics']
        args.gripper_encoder_names = yaml_data['dataInfo']['gripper']['encoder']['names']
        args.gripper_encoder_topics = yaml_data['dataInfo']['gripper']['encoder']['topics']
        args.imu_9axis_names = yaml_data['dataInfo']['imu']['9axis']['names']
        args.imu_9axis_topics = yaml_data['dataInfo']['imu']['9axis']['topics']
        args.lidar_point_cloud_names = yaml_data['dataInfo']['lidar']['pointCloud']['names']
        args.lidar_point_cloud_topics = yaml_data['dataInfo']['lidar']['pointCloud']['topics']
        args.robot_base_vel_names = yaml_data['dataInfo']['robotBase']['vel']['names']
        args.robot_base_vel_topics = yaml_data['dataInfo']['robotBase']['vel']['topics']
        args.lift_motor_names = yaml_data['dataInfo']['lift']['motor']['names']
        args.lift_motor_topics = yaml_data['dataInfo']['lift']['motor']['topics']

    return args


def main():
    args = get_arguments()
    ros_operator = RosOperator(args)

    process_data(args, ros_operator)


if __name__ == '__main__':
    main()
