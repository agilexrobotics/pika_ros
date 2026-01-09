#!/usr/bin/env python3
# -*-coding:utf8-*-
# 本文件为控制单个机械臂节点，控制夹爪机械臂运动
from typing import (
    Optional,
)
import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import time
import threading
import argparse
import math
from piper_sdk import *
from piper_sdk import C_PiperInterface
from piper_msgs.msg import PiperStatusMsg, PosCmd
from piper_msgs.srv  import Enable
from geometry_msgs.msg import Pose, PoseStamped
from scipy.spatial.transform import Rotation as R  # 用于欧拉角到四元数的转换

class C_PiperRosNode(Node):
    """机械臂ros2节点
    """
    def __init__(self) -> None:
        super().__init__('piper_ctrl_single_node')
        # 外部param参数
        # 外部参数
        self.declare_parameter('can_port', 'can0')
        self.declare_parameter('auto_enable', False)
        self.declare_parameter('girpper_exist', True)
        self.declare_parameter('rviz_ctrl_flag', False)
        self.declare_parameter('piper_name', 'left')

        self.can_port = self.get_parameter('can_port').get_parameter_value().string_value
        self.auto_enable = self.get_parameter('auto_enable').get_parameter_value().bool_value
        self.girpper_exist = self.get_parameter('girpper_exist').get_parameter_value().bool_value
        self.rviz_ctrl_flag = self.get_parameter('rviz_ctrl_flag').get_parameter_value().bool_value	
        self.piper_name = self.get_parameter('piper_name').get_parameter_value().string_value

        self.get_logger().info(f"can_port is {self.can_port}")
        self.get_logger().info(f"auto_enable is {self.auto_enable}")
        self.get_logger().info(f"girpper_exist is {self.girpper_exist}")
        self.get_logger().info(f"rviz_ctrl_flag is {self.rviz_ctrl_flag}")
        self.get_logger().info(f"piper_name is {self.piper_name}")
        # Publisher
        self.joint_pub = self.create_publisher(JointState, f'/puppet/joint_{self.piper_name}', 1)
        self.joint_ctrl_pub = self.create_publisher(JointState, f'/master/joint_{self.piper_name}', 1)
        self.arm_status_pub = self.create_publisher(PiperStatusMsg, self.get_name()+'/arm_status', 1)
        self.end_pose_pub = self.create_publisher(Pose, self.get_name()+'/end_pose', 1)
        self.end_pose_stamp_pub = self.create_publisher(PoseStamped, f'/puppet/end_pose_{self.piper_name}', 1)
        # service
        self.motor_srv = self.create_service(Enable, 'enable_srv', self.handle_enable_service)
        # joint
        self.joint_states = JointState()
        self.joint_states.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.joint_states.position = [0.0] * 7
        self.joint_states.velocity = [0.0] * 7
        self.joint_states.effort = [0.0] * 7
        # Joint ctrl
        self.joint_ctrl = JointState()
        self.joint_ctrl.name = ['joint0', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.joint_ctrl.position = [0.0] * 7
        self.joint_ctrl.velocity = [0.0] * 7
        self.joint_ctrl.effort = [0.0] * 7
        # 使能标志位
        self.__enable_flag = False
        # 创建piper类并打开can接口
        self.piper = C_PiperInterface(can_name=self.can_port)
        self.piper.ConnectPort()

        # 启动订阅线程
        self.create_subscription(PosCmd, 'pos_cmd', self.pos_callback, 1)
        self.create_subscription(JointState, 'joint_ctrl_single', self.joint_callback, 1)
        self.create_subscription(Bool, 'enable_flag', self.enable_callback, 1)
        
        self.publisher_thread = threading.Thread(target=self.publish_thread)
        self.publisher_thread.start()
    def get_private_topic(self, topic_name):
        return self.get_name()+'/'+topic_name

    def GetEnableFlag(self):
        return self.__enable_flag

    def publish_thread(self):
        """机械臂消息发布
        """
        rate = self.create_rate(200)  # 200 Hz
        enable_flag = False
        # 设置超时时间（秒）
        timeout = 5
        # 记录进入循环前的时间
        start_time = time.time()
        elapsed_time_flag = False
        while rclpy.ok():
            # print(self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status)
            # print(self.piper.GetArmEndPoseMsgs())
            if(self.auto_enable):
                while not (enable_flag):
                    elapsed_time = time.time() - start_time
                    print("--------------------")
                    enable_flag = self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status and \
                        self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status
                    print("使能状态:",enable_flag)
                    self.piper.EnableArm(7)
                    self.piper.GripperCtrl(0,1000,0x02, 0)
                    self.piper.GripperCtrl(0,1000,0x01, 0)
                    if(enable_flag):
                        self.__enable_flag = True
                    print("--------------------")
                    # 检查是否超过超时时间
                    if elapsed_time > timeout:
                        print("超时....")
                        elapsed_time_flag = True
                        enable_flag = True
                        break
                    time.sleep(1)
                    pass
            if(elapsed_time_flag):
                print("程序自动使能超时,退出程序")
                exit(0)
            
            self.PublishArmState()
            self.PublishArmJointAndGirpper()
            self.PublishArmCtrlAndGripper()
            self.PubilsArmEndPose()
            
            rate.sleep()

    def PublishArmState(self):
        arm_status = PiperStatusMsg()
        arm_status.ctrl_mode = self.piper.GetArmStatus().arm_status.ctrl_mode
        arm_status.arm_status = self.piper.GetArmStatus().arm_status.arm_status
        arm_status.mode_feedback = self.piper.GetArmStatus().arm_status.mode_feed
        arm_status.teach_status = self.piper.GetArmStatus().arm_status.teach_status
        arm_status.motion_status = self.piper.GetArmStatus().arm_status.motion_status
        arm_status.trajectory_num = self.piper.GetArmStatus().arm_status.trajectory_num
        arm_status.err_code = self.piper.GetArmStatus().arm_status.err_code
        arm_status.joint_1_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_1_angle_limit
        arm_status.joint_2_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_2_angle_limit
        arm_status.joint_3_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_3_angle_limit
        arm_status.joint_4_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_4_angle_limit
        arm_status.joint_5_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_5_angle_limit
        arm_status.joint_6_angle_limit = self.piper.GetArmStatus().arm_status.err_status.joint_6_angle_limit
        arm_status.communication_status_joint_1 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_1
        arm_status.communication_status_joint_2 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_2
        arm_status.communication_status_joint_3 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_3
        arm_status.communication_status_joint_4 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_4
        arm_status.communication_status_joint_5 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_5
        arm_status.communication_status_joint_6 = self.piper.GetArmStatus().arm_status.err_status.communication_status_joint_6
        self.arm_status_pub.publish(arm_status)
    
    def PublishArmJointAndGirpper(self):
        # 赋值时间戳
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        # Here, you can set the joint positions to any value you want
        # 由于获取的原始数据是度为单位扩大了1000倍，因此要转为弧度需要先除以1000，再乘3.14/180，然后限制小数点位数为5位
        joint_0:float = (self.piper.GetArmJointMsgs().joint_state.joint_1/1000) * 3.141592653589 / 180
        joint_1:float = (self.piper.GetArmJointMsgs().joint_state.joint_2/1000) * 3.141592653589 / 180
        joint_2:float = (self.piper.GetArmJointMsgs().joint_state.joint_3/1000) * 3.141592653589 / 180
        joint_3:float = (self.piper.GetArmJointMsgs().joint_state.joint_4/1000) * 3.141592653589 / 180
        joint_4:float = (self.piper.GetArmJointMsgs().joint_state.joint_5/1000) * 3.141592653589 / 180
        joint_5:float = (self.piper.GetArmJointMsgs().joint_state.joint_6/1000) * 3.141592653589 / 180
        joint_6:float = self.piper.GetArmGripperMsgs().gripper_state.grippers_angle/1000000
        vel_0:float = self.piper.GetArmHighSpdInfoMsgs().motor_1.motor_speed/1000
        vel_1:float = self.piper.GetArmHighSpdInfoMsgs().motor_2.motor_speed/1000
        vel_2:float = self.piper.GetArmHighSpdInfoMsgs().motor_3.motor_speed/1000
        vel_3:float = self.piper.GetArmHighSpdInfoMsgs().motor_4.motor_speed/1000
        vel_4:float = self.piper.GetArmHighSpdInfoMsgs().motor_5.motor_speed/1000
        vel_5:float = self.piper.GetArmHighSpdInfoMsgs().motor_6.motor_speed/1000
        effort_6:float = self.piper.GetArmGripperMsgs().gripper_state.grippers_effort/1000
        self.joint_states.position = [joint_0,joint_1, joint_2, joint_3, joint_4, joint_5,joint_6]  # Example values
        self.joint_states.velocity = [vel_0, vel_1, vel_2, vel_3, vel_4, vel_5, 0.0]  # Example values
        self.joint_states.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, effort_6]
        self.joint_pub.publish(self.joint_states)

    def PublishArmCtrlAndGripper(self):
        self.joint_ctrl.header.stamp = self.get_clock().now().to_msg()
        joint_0: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_1/1000) * 0.017444
        joint_1: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_2/1000) * 0.017444
        joint_2: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_3/1000) * 0.017444
        joint_3: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_4/1000) * 0.017444
        joint_4: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_5/1000) * 0.017444
        joint_5: float = (self.piper.GetArmJointCtrl().joint_ctrl.joint_6/1000) * 0.017444
        joint_6: float = self.piper.GetArmGripperCtrl().gripper_ctrl.grippers_angle * 0.001
        self.joint_ctrl.position = [joint_0, joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]  # Example values
        self.joint_ctrl_pub.publish(self.joint_ctrl)

    def PubilsArmEndPose(self):
        # 末端位姿
        endpos = Pose()
        endpos.position.x = self.piper.GetArmEndPoseMsgs().end_pose.X_axis/1000000
        endpos.position.y = self.piper.GetArmEndPoseMsgs().end_pose.Y_axis/1000000
        endpos.position.z = self.piper.GetArmEndPoseMsgs().end_pose.Z_axis/1000000
        roll = self.piper.GetArmEndPoseMsgs().end_pose.RX_axis/1000
        pitch = self.piper.GetArmEndPoseMsgs().end_pose.RY_axis/1000
        yaw = self.piper.GetArmEndPoseMsgs().end_pose.RZ_axis/1000
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        quaternion = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
        endpos.orientation.x = quaternion[0]
        endpos.orientation.y = quaternion[1]
        endpos.orientation.z = quaternion[2]
        endpos.orientation.w = quaternion[3]
        self.end_pose_pub.publish(endpos)

        end_pos_stamp = PoseStamped()
        end_pos_stamp.pose = endpos
        end_pos_stamp.header.stamp = self.get_clock().now().to_msg()
        self.end_pose_stamp_pub.publish(end_pos_stamp)

    
    def pos_callback(self, pos_data):
        """机械臂末端位姿订阅回调函数

        Args:
            pos_data (): 
        """
        factor = 180 / 3.1415926
        self.get_logger().info(f"Received PosCmd:")
        self.get_logger().info(f"x: {pos_data.x}")
        self.get_logger().info(f"y: {pos_data.y}")
        self.get_logger().info(f"z: {pos_data.z}")
        self.get_logger().info(f"roll: {pos_data.roll}")
        self.get_logger().info(f"pitch: {pos_data.pitch}")
        self.get_logger().info(f"yaw: {pos_data.yaw}")
        self.get_logger().info(f"gripper: {pos_data.gripper}")
        self.get_logger().info(f"mode1: {pos_data.mode1}")
        self.get_logger().info(f"mode2: {pos_data.mode2}")
        x = round(pos_data.x*1000) * 1000
        y = round(pos_data.y*1000) * 1000
        z = round(pos_data.z*1000) * 1000
        rx = round(pos_data.roll*1000*factor)
        ry = round(pos_data.pitch*1000*factor)
        rz = round(pos_data.yaw*1000*factor)
        if(self.GetEnableFlag()):
            self.piper.MotionCtrl_1(0x00, 0x00, 0x00)
            self.piper.MotionCtrl_2(0x01, 0x02, 50)
            self.piper.EndPoseCtrl(x, y, z, 
                                    rx, ry, rz)
            gripper = round(pos_data.gripper*1000*1000)
            if(pos_data.gripper>80000): gripper = 80000
            if(pos_data.gripper<0): gripper = 0
            if(self.girpper_exist):
                self.piper.GripperCtrl(abs(gripper), 1000, 0x01, 0)
            self.piper.MotionCtrl_2(0x01, 0x00, 50)
    
    def joint_callback(self, joint_data):
        """机械臂关节角回调函数

        Args:
            joint_data (): 
        """
        factor = 57324.840764 #1000*180/3.14
        #self.get_logger().info(f"Received Joint States:")
        #self.get_logger().info(f"joint_0: {joint_data.position[0]}")
        #self.get_logger().info(f"joint_1: {joint_data.position[1]}")
        #self.get_logger().info(f"joint_2: {joint_data.position[2]}")
        #self.get_logger().info(f"joint_3: {joint_data.position[3]}")
        #self.get_logger().info(f"joint_4: {joint_data.position[4]}")
        #self.get_logger().info(f"joint_5: {joint_data.position[5]}")
        #self.get_logger().info(f"joint_6: {joint_data.position[6]}")
        joint_0 = round(joint_data.position[0]*factor)
        joint_1 = round(joint_data.position[1]*factor)
        joint_2 = round(joint_data.position[2]*factor)
        joint_3 = round(joint_data.position[3]*factor)
        joint_4 = round(joint_data.position[4]*factor)
        joint_5 = round(joint_data.position[5]*factor)
        joint_6 = round(joint_data.position[6]*1000*1000)
        if(len(joint_data.position) >= 7):
            joint_6 = round(joint_data.position[6]*1000*1000)
            if(self.rviz_ctrl_flag):
                joint_6 = joint_6 * 2
            if(joint_6>80000): joint_6 = 80000
            if(joint_6<0): joint_6 = 0
        else: joint_6 = 0
        if(self.GetEnableFlag()):
            # 设定电机速度
            if(joint_data.velocity != []):
                all_zeros = all(v == 0 for v in joint_data.velocity)
            else: all_zeros = True
            if(not all_zeros):
                lens = len(joint_data.velocity)
                if(lens == 7):
                    vel_all = round(joint_data.velocity[6])
                    if (vel_all > 100): vel_all = 100
                    if (vel_all <= 30): vel_all = 30
                    # self.get_logger().info(f"vel_all: {vel_all}")
                    self.piper.MotionCtrl_2(0x01, 0x01, vel_all)
                # elif(lens == 7):
                #     # 遍历速度列表
                #     for i, velocity in enumerate(joint_data.velocity):
                #         if velocity > 0:  # 如果速度是正数
                #             # 设置指定位置的关节速度为这个正数速度
                #             # self.piper.SearchMotorMaxAngleSpdAccLimit(i+1,0x01)
                #             # self.piper.MotorAngleLimitMaxSpdSet(i+1)
                else: self.piper.MotionCtrl_2(0x01, 0x01, 30)
            else: self.piper.MotionCtrl_2(0x01, 0x01, 30)
            self.piper.JointCtrl(joint_0, joint_1, joint_2, 
                                    joint_3, joint_4, joint_5)
            if(self.girpper_exist):
                if(len(joint_data.effort) >= 7):
                    gripper_effort = joint_data.effort[6]
                    if (gripper_effort > 3): gripper_effort = 3
                    if (gripper_effort < 0.5): gripper_effort = 0.5
                    # self.get_logger().info(f"gripper_pos: {joint_6}")
                    # self.get_logger().info(f"gripper_effort: {gripper_effort}")
                    gripper_effort = round(gripper_effort*1000)
                    self.piper.GripperCtrl(abs(joint_6), gripper_effort, 0x01, 0)
                # 默认1N
                else: self.piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)
    
    def enable_callback(self, enable_flag:Bool):
        """机械臂使能回调函数

        Args:
            enable_flag (): 
        """
        self.get_logger().info(f"Received enable flag:")
        self.get_logger().info(f"enable_flag: {enable_flag.data}")
        if(enable_flag.data):
            self.__enable_flag = True
            self.piper.EnableArm(7)
            if(self.girpper_exist):
                self.piper.GripperCtrl(0,1000,0x01, 0)
        else:
            self.__enable_flag = False
            self.piper.DisableArm(7)
            if(self.girpper_exist):
                self.piper.GripperCtrl(0,1000,0x00, 0)

    def handle_enable_service(self, req, resp):
        self.get_logger().info(f"Received request:: {req.enable_request}")
        enable_flag = False
        loop_flag = False
        # 设置超时时间（秒）
        timeout = 5
        # 记录进入循环前的时间
        start_time = time.time()
        elapsed_time_flag = False
        while not (loop_flag):
            elapsed_time = time.time() - start_time
            self.get_logger().info(f"--------------------")
            enable_list = []
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_1.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_2.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_3.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_4.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_5.foc_status.driver_enable_status)
            enable_list.append(self.piper.GetArmLowSpdInfoMsgs().motor_6.foc_status.driver_enable_status)
            if(req.enable_request):
                enable_flag = all(enable_list)
                self.piper.EnableArm(7)
                self.piper.GripperCtrl(0,1000,0x01, 0)
            else:
                enable_flag = any(enable_list)
                self.piper.DisableArm(7)
                self.piper.GripperCtrl(0,1000,0x02, 0)
            self.get_logger().info(f"使能状态: {enable_flag}")
            self.__enable_flag = enable_flag
            self.get_logger().info(f"--------------------")
            if(enable_flag == req.enable_request):
                loop_flag = True
                enable_flag = True
            else: 
                loop_flag = False
                enable_flag = False
            # 检查是否超过超时时间
            if elapsed_time > timeout:
                self.get_logger().info(f"超时....")
                elapsed_time_flag = True
                enable_flag = False
                loop_flag = True
                break
            time.sleep(0.5)
        resp.enable_response = enable_flag
        self.get_logger().info(f"Returning response: {resp.enable_response}")
        return resp
    
def main(args=None):
    rclpy.init(args=args)
    piper_single_node = C_PiperRosNode()
    try:
        rclpy.spin(piper_single_node)
    except KeyboardInterrupt:
        pass
    finally:
        piper_single_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
