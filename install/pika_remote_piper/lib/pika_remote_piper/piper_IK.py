#!/usr/bin/env python3
import casadi
import meshcat.geometry as mg
import math
import numpy as np
import pinocchio as pin
from pinocchio import casadi as cpin
from pinocchio.visualize import MeshcatVisualizer
import os
import sys
import cv2
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import argparse
import time
from data_msgs.msg import ArmControlStatus
import rclpy
from rclpy.node import Node
import ast
import threading

# Add current script directory to Python path to find local modules
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT_DIR)
sys.path.append(os.path.join(ROOT_DIR,'dist_pyarmor_FIK'))

from transformations import quaternion_from_euler, euler_from_quaternion, quaternion_from_matrix
from arm_FIK import Arm_IK

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
        super().__init__(f'piper_IK{args.index_name}')
        self.args = args
        self.arm_end_pose_subscriber = None
        self.arm_joint_state_publisher = None
        self.arm_end_pose_publisher = None
        self.arm_end_pose_orient_publisher = None
        self.arm_receive_end_pose_publisher = None
        self.lift_publisher = None
        self.last_ctrl_arm_joint_state = None
        self.arm_joint_state = None
        self.publish_lock = threading.Lock()
        self.publish_thread = None
        self.sol_q = None
        self.msg = None
        self.xyzrpy = None
        self.arm_ik = Arm_IK(args)
        self.init_ros()

    def arm_joint_state_ctrl(self, joint_state):
        self.last_ctrl_arm_joint_state = joint_state[:6]
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.position = [float(v) for v in joint_state]
        joint_state_msg.velocity = [float(1) for v in joint_state]
        joint_state_msg.effort = [float(1) for v in joint_state]
        self.arm_joint_state_publisher.publish(joint_state_msg)

    def arm_joint_state_ctrl_linear_interpolation(self, joint_state):
        if self.last_ctrl_arm_joint_state is None:
            self.last_ctrl_arm_joint_state = self.arm_joint_state
        joint_state_diff = max(abs(self.last_ctrl_arm_joint_state - joint_state[:6]))
        if joint_state_diff > 30.0 / 180 * 3.1415926:
            print("max_diff:", joint_state_diff / 3.1415926 * 180)
            step = int(joint_state_diff / (1.0 / 180 * 3.1415926))
            hz = 200
            rate = self.create_rate(hz)
            joint_state_list = np.linspace(self.last_ctrl_arm_joint_state, joint_state[:6], step + 1)
            for i in range(1, len(joint_state_list)):
                joint_state_inter = joint_state_list[i].tolist()
                if len(joint_state) > 6:
                    joint_state_inter = joint_state_list[i].tolist()
                    joint_state_inter.append(joint_state[-1])
                self.arm_joint_state_ctrl(np.array(joint_state_inter))
                rate.sleep()
            return
        else:
            self.arm_joint_state_ctrl(joint_state)

    def arm_joint_state_callback(self, msg):
        self.arm_joint_state = np.array(msg.position[:6])

    def publishing(self):
        rate = self.create_rate(50)
        while rclpy.ok():
            self.publish_lock.acquire() 
            msg, sol_q, xyzrpy = self.msg, self.sol_q, self.xyzrpy
            self.publish_lock.release() 
            if msg is None or sol_q is None or xyzrpy is None:
                rate.sleep()
                continue
            count = 0
            if self.args.lift:
                joint_position = []
                joint_position.append(sol_q[0])
                joint_state_msg = JointState()
                joint_state_msg.header = Header()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = [f'joint1']
                joint_state_msg.position = joint_position
                self.lift_publisher.publish(joint_state_msg)
                count += 1

            joint_position = []
            joint_position.append(sol_q[count + 0])
            joint_position.append(sol_q[count + 1])
            joint_position.append(sol_q[count + 2])
            joint_position.append(sol_q[count + 3])
            joint_position.append(sol_q[count + 4])
            joint_position.append(sol_q[count + 5])

            joint_state_msg = JointState()
            joint_state_msg.header = Header()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = [f'joint{i+1}' for i in range(8)]
            joint_state_msg.position = joint_position
            if not self.args.use_orient:
                # joint_state_msg.position.append(msg.pose.orientation.w / 2.0)
                # joint_state_msg.position.append(-msg.pose.orientation.w / 2.0)
                joint_state_msg.position.append(msg.pose.orientation.w)
            # self.arm_joint_state_publisher.publish(joint_state_msg)
            self.arm_joint_state_ctrl_linear_interpolation(np.array(joint_state_msg.position))

            xyzrpy = self.arm_ik.get_pose(sol_q[:6+(1 if self.args.lift else 0)])

            if not self.args.use_orient:
                end_pose_msg = PoseStamped()
                end_pose_msg.header = Header()
                end_pose_msg.header.stamp = self.get_clock().now().to_msg()
                end_pose_msg.header.frame_id = "map"
                end_pose_msg.pose.position.x = xyzrpy[0]
                end_pose_msg.pose.position.y = xyzrpy[1]
                end_pose_msg.pose.position.z = xyzrpy[2]
                end_pose_msg.pose.orientation.x = xyzrpy[3]
                end_pose_msg.pose.orientation.y = xyzrpy[4]
                end_pose_msg.pose.orientation.z = xyzrpy[5]
                # end_pose_msg.pose.orientation.w = joint_position[-2] - joint_position[-1]
                end_pose_msg.pose.orientation.w = msg.pose.orientation.w
                self.arm_end_pose_publisher.publish(end_pose_msg)

            x, y, z, w = quaternion_from_euler(xyzrpy[3], xyzrpy[4], xyzrpy[5])
            end_pose_msg = PoseStamped()
            end_pose_msg.header = Header()
            end_pose_msg.header.stamp = self.get_clock().now().to_msg()
            end_pose_msg.header.frame_id = "map"
            end_pose_msg.pose.position.x = xyzrpy[0]
            end_pose_msg.pose.position.y = xyzrpy[1]
            end_pose_msg.pose.position.z = xyzrpy[2]
            end_pose_msg.pose.orientation.x = x
            end_pose_msg.pose.orientation.y = y
            end_pose_msg.pose.orientation.z = z
            end_pose_msg.pose.orientation.w = w
            self.arm_end_pose_orient_publisher.publish(end_pose_msg)

            if not self.args.use_orient:
                end_pose_msg.pose.position.x = msg.pose.position.x
                end_pose_msg.pose.position.y = msg.pose.position.y
                end_pose_msg.pose.position.z = msg.pose.position.z
                end_pose_msg.pose.orientation.x = msg.pose.orientation.x
                end_pose_msg.pose.orientation.y = msg.pose.orientation.y
                end_pose_msg.pose.orientation.z = msg.pose.orientation.z
                x, y, z, w = quaternion_from_euler(end_pose_msg.pose.orientation.x, end_pose_msg.pose.orientation.y, end_pose_msg.pose.orientation.z)
                end_pose_msg.pose.orientation.x = x
                end_pose_msg.pose.orientation.y = y
                end_pose_msg.pose.orientation.z = z
                end_pose_msg.pose.orientation.w = w
                self.arm_receive_end_pose_publisher.publish(end_pose_msg)
            rate.sleep()

    def arm_end_pose_callback(self, msg):
        if self.last_ctrl_arm_joint_state is None and self.arm_joint_state is None:
            print("check joint_state topic")
            return
        # print(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        if self.args.use_orient:
            target = pin.SE3(
                pin.Quaternion(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z),
                np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]),
            )
        else:
            q = quaternion_from_euler(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)
            target = pin.SE3(
                pin.Quaternion(q[3], q[0], q[1], q[2]),
                np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]),
            )
        sol_q, tau_ff, get_result = self.arm_ik.ik_fun(target.homogeneous, msg.pose.orientation.w)
        if get_result:
            xyzrpy = self.arm_ik.get_pose(sol_q[:6+(1 if self.args.lift else 0)])
            diffX = abs(xyzrpy[0] - msg.pose.position.x)
            diffY = abs(xyzrpy[1] - msg.pose.position.y)
            diffZ = abs(xyzrpy[2] - msg.pose.position.z)
            diffRoll = abs(xyzrpy[3] - msg.pose.orientation.x)
            diffPitch = abs(xyzrpy[4] - msg.pose.orientation.y)
            diffYaw = abs(xyzrpy[5] - msg.pose.orientation.z)
            # print("diff:", diffX, diffY, diffZ, diffRoll, diffPitch, diffYaw)
            if diffX > 0.3 or diffY > 0.3 or diffZ > 0.3 or diffRoll > 1 or diffPitch > 1 or diffYaw > 1:
                get_result = False        # print("result:", sol_q, tau_ff)
        if get_result:
            status_msg = ArmControlStatus()
            status_msg.over_limit = False
            self.arm_control_status_publisher.publish(status_msg)
            self.publish_lock.acquire()
            self.msg, self.sol_q, self.xyzrpy = msg, sol_q, xyzrpy
            self.publish_lock.release()
        else:
            status_msg = ArmControlStatus()
            status_msg.over_limit = True
            self.arm_control_status_publisher.publish(status_msg)

    def init_ros(self):
        self.declare_parameter('index_name', "")
        self.declare_parameter('gripper_xyzrpy', "[0.19, 0, 0, 0, 0, 0]")
        self.args.index_name = self.get_parameter('index_name').get_parameter_value().string_value
        self.args.gripper_xyzrpy = self.get_parameter('gripper_xyzrpy').get_parameter_value().string_value
        self.args.gripper_xyzrpy = ast.literal_eval(self.args.gripper_xyzrpy)
        self.arm_end_pose_subscriber = self.create_subscription(PoseStamped, f'/piper_IK{self.args.index_name}/ctrl_end_pose', self.arm_end_pose_callback, 1)
        self.arm_joint_state_subscriber = self.create_subscription(JointState, f'/joint_states_single{self.args.index_name}', self.arm_joint_state_callback, 1)
        self.arm_joint_state_publisher = self.create_publisher(JointState, f'/joint_states{self.args.index_name}', 1)
        self.arm_end_pose_orient_publisher = self.create_publisher(PoseStamped, f'/piper_IK{self.args.index_name}/urdf_end_pose_orient', 1)
        self.arm_control_status_publisher = self.create_publisher(ArmControlStatus, f'/arm_control_status{self.args.index_name}', 1)
        if not self.args.use_orient:
            self.arm_end_pose_publisher = self.create_publisher(PoseStamped, f'/piper_IK{self.args.index_name}/urdf_end_pose', 1)
            self.arm_receive_end_pose_publisher = self.create_publisher(PoseStamped, f'/piper_IK{self.args.index_name}/receive_end_pose_orient', 1)
        if self.args.lift:
            self.lift_publisher = self.create_publisher(JointState, f'/joint_states_lift', queue_size=1)
        self.publish_thread = threading.Thread(target=self.publishing)
        self.publish_thread.start()


def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--use_orient', action='store', type=str, help='use_orient',
                        default=False, required=False)
    parser.add_argument('--lift', action='store', type=bool, help='lift',
                        default=False, required=False)
    parser.add_argument('--index_name', action='store', type=str, help='index_name',
                        default="", required=False)
    parser.add_argument('--gripper_xyzrpy', action='store', nargs='+', type=float, help='gripper_xyzrpy',
                        default=[0.19, 0, 0, 0, 0, 0], required=False)
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