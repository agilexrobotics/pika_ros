#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Data Synchronization Tool (Python Version)
This script synchronizes multi-modal sensor data based on timestamps,
converted from the original C++ ROS implementation.
"""

import os
import sys
import argparse
import yaml
import json
import math
import numpy as np
from typing import List, Dict, Tuple, Optional


class TimeSeries:
    """时间序列数据项"""
    def __init__(self, time: float, data_list: List, sync_list: List):
        self.time = time
        self.data_list = data_list
        self.sync_list = sync_list

    def to_data_list(self):
        """将数据添加到数据列表"""
        self.data_list.append(self)

    def to_sync_list(self):
        """将数据添加到同步列表"""
        self.sync_list.append(self)


class Operator:
    """数据同步器"""
    
    def __init__(self, args):
        self.args = args
        # 初始化目录路径
        self.init_directories()
        
        # 初始化时间序列存储
        self.init_time_series()
        
        # 存储所有时间戳
        self.all_time_series: List[TimeSeries] = []

    def init_directories(self):
        """初始化数据目录路径"""
        self.episode_dir = os.path.join(self.args.datasetDir, self.args.episodeName)
        
        # 构建各类数据的目录路径
        self.camera_color_dirs = [os.path.join(self.episode_dir, "camera/color", name) for name in self.args.cameraColorNames]
        self.camera_depth_dirs = [os.path.join(self.episode_dir, "camera/depth", name) for name in self.args.cameraDepthNames]
        self.camera_point_cloud_dirs = [os.path.join(self.episode_dir, "camera/pointCloud", name) for name in self.args.cameraPointCloudNames]
        self.arm_joint_state_dirs = [os.path.join(self.episode_dir, "arm/jointState", name) for name in self.args.armJointStateNames]
        self.arm_end_pose_dirs = [os.path.join(self.episode_dir, "arm/endPose", name) for name in self.args.armEndPoseNames]
        self.localization_pose_dirs = [os.path.join(self.episode_dir, "localization/pose", name) for name in self.args.localizationPoseNames]
        self.gripper_encoder_dirs = [os.path.join(self.episode_dir, "gripper/encoder", name) for name in self.args.gripperEncoderNames]
        self.imu_9axis_dirs = [os.path.join(self.episode_dir, "imu/9axis", name) for name in self.args.imu9AxisNames]
        self.lidar_point_cloud_dirs = [os.path.join(self.episode_dir, "lidar/pointCloud", name) for name in self.args.lidarPointCloudNames]
        self.robot_base_vel_dirs = [os.path.join(self.episode_dir, "robotBase/vel", name) for name in self.args.robotBaseVelNames]
        self.lift_motor_dirs = [os.path.join(self.episode_dir, "lift/motor", name) for name in self.args.liftMotorNames]

    def init_time_series(self):
        """初始化时间序列存储结构"""
        # 数据时间序列
        self.camera_color_data_time_series = [[] for _ in self.args.cameraColorNames]
        self.camera_depth_data_time_series = [[] for _ in self.args.cameraDepthNames]
        self.camera_point_cloud_data_time_series = [[] for _ in self.args.cameraPointCloudNames]
        self.arm_joint_state_data_time_series = [[] for _ in self.args.armJointStateNames]
        self.arm_end_pose_data_time_series = [[] for _ in self.args.armEndPoseNames]
        self.localization_pose_data_time_series = [[] for _ in self.args.localizationPoseNames]
        self.gripper_encoder_data_time_series = [[] for _ in self.args.gripperEncoderNames]
        self.imu_9axis_data_time_series = [[] for _ in self.args.imu9AxisNames]
        self.lidar_point_cloud_data_time_series = [[] for _ in self.args.lidarPointCloudNames]
        self.robot_base_vel_data_time_series = [[] for _ in self.args.robotBaseVelNames]
        self.lift_motor_data_time_series = [[] for _ in self.args.liftMotorNames]
        
        # 同步时间序列
        self.camera_color_sync_time_series = [[] for _ in self.args.cameraColorNames]
        self.camera_depth_sync_time_series = [[] for _ in self.args.cameraDepthNames]
        self.camera_point_cloud_sync_time_series = [[] for _ in self.args.cameraPointCloudNames]
        self.arm_joint_state_sync_time_series = [[] for _ in self.args.armJointStateNames]
        self.arm_end_pose_sync_time_series = [[] for _ in self.args.armEndPoseNames]
        self.localization_pose_sync_time_series = [[] for _ in self.args.localizationPoseNames]
        self.gripper_encoder_sync_time_series = [[] for _ in self.args.gripperEncoderNames]
        self.imu_9axis_sync_time_series = [[] for _ in self.args.imu9AxisNames]
        self.lidar_point_cloud_sync_time_series = [[] for _ in self.args.lidarPointCloudNames]
        self.robot_base_vel_sync_time_series = [[] for _ in self.args.robotBaseVelNames]
        self.lift_motor_sync_time_series = [[] for _ in self.args.liftMotorNames]
        
        # 文件扩展名
        self.camera_color_exts = [".jpg"] * len(self.args.cameraColorNames)
        self.camera_depth_exts = [".png"] * len(self.args.cameraDepthNames)
        self.camera_point_cloud_exts = [".pcd"] * len(self.args.cameraPointCloudNames)
        self.arm_joint_state_exts = [".json"] * len(self.args.armJointStateNames)
        self.arm_end_pose_exts = [".json"] * len(self.args.armEndPoseNames)
        self.localization_pose_exts = [".json"] * len(self.args.localizationPoseNames)
        self.gripper_encoder_exts = [".json"] * len(self.args.gripperEncoderNames)
        self.imu_9axis_exts = [".json"] * len(self.args.imu9AxisNames)
        self.lidar_point_cloud_exts = [".json"] * len(self.args.lidarPointCloudNames)
        self.robot_base_vel_exts = [".json"] * len(self.args.robotBaseVelNames)
        self.lift_motor_exts = [".json"] * len(self.args.liftMotorNames)

    def get_files_in_path(self, path: str, ext: str, data_list: List, sync_list: List) -> int:
        """获取指定路径下指定扩展名的文件，并按时间戳排序"""
        count = 0
        if not os.path.exists(path):
            print(f"Warning: Directory {path} does not exist")
            return count
            
        for filename in os.listdir(path):
            if filename.endswith(ext):
                try:
                    # 从文件名提取时间戳
                    timestamp = float(filename[:filename.rfind(".")])
                    time_series = TimeSeries(timestamp, data_list, sync_list)
                    self.all_time_series.append(time_series)
                    count += 1
                except ValueError:
                    # 跳过无法解析时间戳的文件
                    continue
        return count

    def load_all_time_series(self):
        """加载所有数据源的时间序列"""
        
        # 加载各类数据的时间戳
        for i, name in enumerate(self.args.cameraColorNames):
            count = self.get_files_in_path(self.camera_color_dirs[i], ".jpg", 
                                         self.camera_color_data_time_series[i], 
                                         self.camera_color_sync_time_series[i])
            if count == 0:
                count = self.get_files_in_path(self.camera_color_dirs[i], ".png", 
                                             self.camera_color_data_time_series[i], 
                                             self.camera_color_sync_time_series[i])
                if count > 0:
                    self.camera_color_exts[i] = ".png"

        for i, name in enumerate(self.args.cameraDepthNames):
            count = self.get_files_in_path(self.camera_depth_dirs[i], ".png", 
                                         self.camera_depth_data_time_series[i], 
                                         self.camera_depth_sync_time_series[i])

        for i, name in enumerate(self.args.cameraPointCloudNames):
            count = self.get_files_in_path(self.camera_point_cloud_dirs[i], ".pcd", 
                                         self.camera_point_cloud_data_time_series[i], 
                                         self.camera_point_cloud_sync_time_series[i])

        for i, name in enumerate(self.args.armJointStateNames):
            count = self.get_files_in_path(self.arm_joint_state_dirs[i], ".json", 
                                         self.arm_joint_state_data_time_series[i], 
                                         self.arm_joint_state_sync_time_series[i])

        for i, name in enumerate(self.args.armEndPoseNames):
            count = self.get_files_in_path(self.arm_end_pose_dirs[i], ".json", 
                                         self.arm_end_pose_data_time_series[i], 
                                         self.arm_end_pose_sync_time_series[i])

        for i, name in enumerate(self.args.localizationPoseNames):
            count = self.get_files_in_path(self.localization_pose_dirs[i], ".json", 
                                         self.localization_pose_data_time_series[i], 
                                         self.localization_pose_sync_time_series[i])

        for i, name in enumerate(self.args.gripperEncoderNames):
            count = self.get_files_in_path(self.gripper_encoder_dirs[i], ".json", 
                                         self.gripper_encoder_data_time_series[i], 
                                         self.gripper_encoder_sync_time_series[i])

        for i, name in enumerate(self.args.imu9AxisNames):
            count = self.get_files_in_path(self.imu_9axis_dirs[i], ".json", 
                                         self.imu_9axis_data_time_series[i], 
                                         self.imu_9axis_sync_time_series[i])

        for i, name in enumerate(self.args.lidarPointCloudNames):
            count = self.get_files_in_path(self.lidar_point_cloud_dirs[i], ".json", 
                                         self.lidar_point_cloud_data_time_series[i], 
                                         self.lidar_point_cloud_sync_time_series[i])

        for i, name in enumerate(self.args.robotBaseVelNames):
            count = self.get_files_in_path(self.robot_base_vel_dirs[i], ".json", 
                                         self.robot_base_vel_data_time_series[i], 
                                         self.robot_base_vel_sync_time_series[i])

        for i, name in enumerate(self.args.liftMotorNames):
            count = self.get_files_in_path(self.lift_motor_dirs[i], ".json", 
                                         self.lift_motor_data_time_series[i], 
                                         self.lift_motor_sync_time_series[i])

        # 按时间戳排序所有时间序列
        self.all_time_series.sort(key=lambda x: x.time)

    def check_data_adequacy(self, print_info: bool = False) -> Optional[float]:
        """检查数据充足性，返回最早的可用时间戳"""
        result = True
        time = float('inf')
        
        # 检查所有数据源是否都有数据
        for i, name in enumerate(self.args.cameraColorNames):
            if len(self.camera_color_data_time_series[i]) == 0:
                if print_info:
                    print(f"Camera color {name} has no data")
                result = False
            else:
                time = min(time, self.camera_color_data_time_series[i][-1].time)
        
        for i, name in enumerate(self.args.cameraDepthNames):
            if len(self.camera_depth_data_time_series[i]) == 0:
                if print_info:
                    print(f"Camera depth {name} has no data")
                result = False
            else:
                time = min(time, self.camera_depth_data_time_series[i][-1].time)
        
        for i, name in enumerate(self.args.cameraPointCloudNames):
            if len(self.camera_point_cloud_data_time_series[i]) == 0:
                if print_info:
                    print(f"Camera point cloud {name} has no data")
                result = False
            else:
                time = min(time, self.camera_point_cloud_data_time_series[i][-1].time)
        
        for i, name in enumerate(self.args.armJointStateNames):
            if len(self.arm_joint_state_data_time_series[i]) == 0:
                if print_info:
                    print(f"Arm joint state {name} has no data")
                result = False
            else:
                time = min(time, self.arm_joint_state_data_time_series[i][-1].time)
        
        for i, name in enumerate(self.args.armEndPoseNames):
            if len(self.arm_end_pose_data_time_series[i]) == 0:
                if print_info:
                    print(f"Arm end pose {name} has no data")
                result = False
            else:
                time = min(time, self.arm_end_pose_data_time_series[i][-1].time)
                
        for i, name in enumerate(self.args.localizationPoseNames):
            if len(self.localization_pose_data_time_series[i]) == 0:
                if print_info:
                    print(f"Localization pose {name} has no data")
                result = False
            else:
                time = min(time, self.localization_pose_data_time_series[i][-1].time)
        
        for i, name in enumerate(self.args.gripperEncoderNames):
            if len(self.gripper_encoder_data_time_series[i]) == 0:
                if print_info:
                    print(f"Gripper encoder {name} has no data")
                result = False
            else:
                time = min(time, self.gripper_encoder_data_time_series[i][-1].time)
        
        for i, name in enumerate(self.args.imu9AxisNames):
            if len(self.imu_9axis_data_time_series[i]) == 0:
                if print_info:
                    print(f"IMU 9-axis {name} has no data")
                result = False
            else:
                time = min(time, self.imu_9axis_data_time_series[i][-1].time)
        
        for i, name in enumerate(self.args.lidarPointCloudNames):
            if len(self.lidar_point_cloud_data_time_series[i]) == 0:
                if print_info:
                    print(f"Lidar point cloud {name} has no data")
                result = False
            else:
                time = min(time, self.lidar_point_cloud_data_time_series[i][-1].time)
        
        for i, name in enumerate(self.args.robotBaseVelNames):
            if len(self.robot_base_vel_data_time_series[i]) == 0:
                if print_info:
                    print(f"Robot base velocity {name} has no data")
                result = False
            else:
                time = min(time, self.robot_base_vel_data_time_series[i][-1].time)
        
        for i, name in enumerate(self.args.liftMotorNames):
            if len(self.lift_motor_data_time_series[i]) == 0:
                if print_info:
                    print(f"Lift motor {name} has no data")
                result = False
            else:
                time = min(time, self.lift_motor_data_time_series[i][-1].time)
        
        return time if result else None

    def find_closest_index(self, data_series: List[TimeSeries], target_time: float) -> Tuple[int, float]:
        """找到最接近目标时间的数据索引"""
        if not data_series:
            return -1, float('inf')
        
        closest_index = 0
        closest_diff = float('inf')
        
        for j, time_series in enumerate(data_series):
            time_diff = abs(time_series.time - target_time)
            if time_diff < closest_diff:
                closest_diff = time_diff
                closest_index = j
        
        return closest_index, closest_diff

    def sync(self):
        """执行数据同步"""
        frame_count = 0
        
        print(f"All time series: {len(self.all_time_series)}")
        
        for time_series in self.all_time_series:
            time_series.to_data_list()
            frame_time = self.check_data_adequacy()
            
            if frame_time is not None:
                # 为每个数据源找到最接近的时间戳
                time_diff_pass = True
                closest_indices = {}
                
                # 检查相机彩色数据
                for i, name in enumerate(self.args.cameraColorNames):
                    if not time_diff_pass:
                        break
                    if self.camera_color_data_time_series[i]:
                        closest_idx, closest_diff = self.find_closest_index(
                            self.camera_color_data_time_series[i], frame_time)
                        if closest_diff > self.args.timeDiffLimit:
                            time_diff_pass = False
                            break
                        closest_indices[f'camera_color_{i}'] = closest_idx
                
                # 检查相机深度数据
                for i, name in enumerate(self.args.cameraDepthNames):
                    if not time_diff_pass:
                        break
                    if self.camera_depth_data_time_series[i]:
                        closest_idx, closest_diff = self.find_closest_index(
                            self.camera_depth_data_time_series[i], frame_time)
                        if closest_diff > self.args.timeDiffLimit:
                            time_diff_pass = False
                            break
                        closest_indices[f'camera_depth_{i}'] = closest_idx
                
                # 检查机械臂关节状态
                for i, name in enumerate(self.args.armJointStateNames):
                    if not time_diff_pass:
                        break
                    if self.arm_joint_state_data_time_series[i]:
                        closest_idx, closest_diff = self.find_closest_index(
                            self.arm_joint_state_data_time_series[i], frame_time)
                        if closest_diff > self.args.timeDiffLimit:
                            time_diff_pass = False
                            break
                        closest_indices[f'arm_joint_state_{i}'] = closest_idx
                
                # 检查定位姿态数据
                for i, name in enumerate(self.args.localizationPoseNames):
                    if not time_diff_pass:
                        break
                    if self.localization_pose_data_time_series[i]:
                        closest_idx, closest_diff = self.find_closest_index(
                            self.localization_pose_data_time_series[i], frame_time)
                        if closest_diff > self.args.timeDiffLimit:
                            time_diff_pass = False
                            break
                        closest_indices[f'localization_pose_{i}'] = closest_idx
                
                # 检查相机点云数据
                for i, name in enumerate(self.args.cameraPointCloudNames):
                    if not time_diff_pass:
                        break
                    if self.camera_point_cloud_data_time_series[i]:
                        closest_idx, closest_diff = self.find_closest_index(
                            self.camera_point_cloud_data_time_series[i], frame_time)
                        if closest_diff > self.args.timeDiffLimit:
                            time_diff_pass = False
                            break
                        closest_indices[f'camera_point_cloud_{i}'] = closest_idx
                
                # 检查机械臂末端位姿数据
                for i, name in enumerate(self.args.armEndPoseNames):
                    if not time_diff_pass:
                        break
                    if self.arm_end_pose_data_time_series[i]:
                        closest_idx, closest_diff = self.find_closest_index(
                            self.arm_end_pose_data_time_series[i], frame_time)
                        if closest_diff > self.args.timeDiffLimit:
                            time_diff_pass = False
                            break
                        closest_indices[f'arm_end_pose_{i}'] = closest_idx
                
                # 检查夹爪编码器数据
                for i, name in enumerate(self.args.gripperEncoderNames):
                    if not time_diff_pass:
                        break
                    if self.gripper_encoder_data_time_series[i]:
                        closest_idx, closest_diff = self.find_closest_index(
                            self.gripper_encoder_data_time_series[i], frame_time)
                        if closest_diff > self.args.timeDiffLimit:
                            time_diff_pass = False
                            break
                        closest_indices[f'gripper_encoder_{i}'] = closest_idx
                
                # 检查IMU 9轴数据
                for i, name in enumerate(self.args.imu9AxisNames):
                    if not time_diff_pass:
                        break
                    if self.imu_9axis_data_time_series[i]:
                        closest_idx, closest_diff = self.find_closest_index(
                            self.imu_9axis_data_time_series[i], frame_time)
                        if closest_diff > self.args.timeDiffLimit:
                            time_diff_pass = False
                            break
                        closest_indices[f'imu_9axis_{i}'] = closest_idx
                
                # 检查激光雷达点云数据
                for i, name in enumerate(self.args.lidarPointCloudNames):
                    if not time_diff_pass:
                        break
                    if self.lidar_point_cloud_data_time_series[i]:
                        closest_idx, closest_diff = self.find_closest_index(
                            self.lidar_point_cloud_data_time_series[i], frame_time)
                        if closest_diff > self.args.timeDiffLimit:
                            time_diff_pass = False
                            break
                        closest_indices[f'lidar_point_cloud_{i}'] = closest_idx
                
                # 检查机器人底盘速度数据
                for i, name in enumerate(self.args.robotBaseVelNames):
                    if not time_diff_pass:
                        break
                    if self.robot_base_vel_data_time_series[i]:
                        closest_idx, closest_diff = self.find_closest_index(
                            self.robot_base_vel_data_time_series[i], frame_time)
                        if closest_diff > self.args.timeDiffLimit:
                            time_diff_pass = False
                            break
                        closest_indices[f'robot_base_vel_{i}'] = closest_idx
                
                # 检查升降电机数据
                for i, name in enumerate(self.args.liftMotorNames):
                    if not time_diff_pass:
                        break
                    if self.lift_motor_data_time_series[i]:
                        closest_idx, closest_diff = self.find_closest_index(
                            self.lift_motor_data_time_series[i], frame_time)
                        if closest_diff > self.args.timeDiffLimit:
                            time_diff_pass = False
                            break
                        closest_indices[f'lift_motor_{i}'] = closest_idx
                
                # 如果时间差检查通过，将数据加入同步列表
                if time_diff_pass:
                    # 添加相机彩色数据到同步列表
                    for i, name in enumerate(self.args.cameraColorNames):
                        if f'camera_color_{i}' in closest_indices:
                            idx = closest_indices[f'camera_color_{i}']
                            self.camera_color_data_time_series[i][idx].to_sync_list()
                            # 删除已处理的数据
                            del self.camera_color_data_time_series[i][:idx+1]
                    
                    # 添加相机深度数据到同步列表
                    for i, name in enumerate(self.args.cameraDepthNames):
                        if f'camera_depth_{i}' in closest_indices:
                            idx = closest_indices[f'camera_depth_{i}']
                            self.camera_depth_data_time_series[i][idx].to_sync_list()
                            del self.camera_depth_data_time_series[i][:idx+1]
                    
                    # 添加相机点云数据到同步列表
                    for i, name in enumerate(self.args.cameraPointCloudNames):
                        if f'camera_point_cloud_{i}' in closest_indices:
                            idx = closest_indices[f'camera_point_cloud_{i}']
                            self.camera_point_cloud_data_time_series[i][idx].to_sync_list()
                            del self.camera_point_cloud_data_time_series[i][:idx+1]
                    
                    # 添加机械臂关节状态到同步列表
                    for i, name in enumerate(self.args.armJointStateNames):
                        if f'arm_joint_state_{i}' in closest_indices:
                            idx = closest_indices[f'arm_joint_state_{i}']
                            self.arm_joint_state_data_time_series[i][idx].to_sync_list()
                            del self.arm_joint_state_data_time_series[i][:idx+1]
                    
                    # 添加机械臂末端位姿到同步列表
                    for i, name in enumerate(self.args.armEndPoseNames):
                        if f'arm_end_pose_{i}' in closest_indices:
                            idx = closest_indices[f'arm_end_pose_{i}']
                            self.arm_end_pose_data_time_series[i][idx].to_sync_list()
                            del self.arm_end_pose_data_time_series[i][:idx+1]
                    
                    # 添加定位姿态数据到同步列表
                    for i, name in enumerate(self.args.localizationPoseNames):
                        if f'localization_pose_{i}' in closest_indices:
                            idx = closest_indices[f'localization_pose_{i}']
                            self.localization_pose_data_time_series[i][idx].to_sync_list()
                            del self.localization_pose_data_time_series[i][:idx+1]
                    
                    # 添加夹爪编码器数据到同步列表
                    for i, name in enumerate(self.args.gripperEncoderNames):
                        if f'gripper_encoder_{i}' in closest_indices:
                            idx = closest_indices[f'gripper_encoder_{i}']
                            self.gripper_encoder_data_time_series[i][idx].to_sync_list()
                            del self.gripper_encoder_data_time_series[i][:idx+1]
                    
                    # 添加IMU 9轴数据到同步列表
                    for i, name in enumerate(self.args.imu9AxisNames):
                        if f'imu_9axis_{i}' in closest_indices:
                            idx = closest_indices[f'imu_9axis_{i}']
                            self.imu_9axis_data_time_series[i][idx].to_sync_list()
                            del self.imu_9axis_data_time_series[i][:idx+1]
                    
                    # 添加激光雷达点云数据到同步列表
                    for i, name in enumerate(self.args.lidarPointCloudNames):
                        if f'lidar_point_cloud_{i}' in closest_indices:
                            idx = closest_indices[f'lidar_point_cloud_{i}']
                            self.lidar_point_cloud_data_time_series[i][idx].to_sync_list()
                            del self.lidar_point_cloud_data_time_series[i][:idx+1]
                    
                    # 添加机器人底盘速度数据到同步列表
                    for i, name in enumerate(self.args.robotBaseVelNames):
                        if f'robot_base_vel_{i}' in closest_indices:
                            idx = closest_indices[f'robot_base_vel_{i}']
                            self.robot_base_vel_data_time_series[i][idx].to_sync_list()
                            del self.robot_base_vel_data_time_series[i][:idx+1]
                    
                    # 添加升降电机数据到同步列表
                    for i, name in enumerate(self.args.liftMotorNames):
                        if f'lift_motor_{i}' in closest_indices:
                            idx = closest_indices[f'lift_motor_{i}']
                            self.lift_motor_data_time_series[i][idx].to_sync_list()
                            del self.lift_motor_data_time_series[i][:idx+1]
                    
                    frame_count += 1

        print(f"Sync frame num: {frame_count}")
        if frame_count == 0:
            self.check_data_adequacy(True)

        # 写入同步文件
        self.write_sync_files()

    def write_sync_files(self):
        """写入同步文件"""
        
        # 写入相机彩色数据同步文件
        for i, name in enumerate(self.args.cameraColorNames):
            sync_file_path = os.path.join(self.camera_color_dirs[i], "sync.txt")
            os.makedirs(os.path.dirname(sync_file_path), exist_ok=True)
            with open(sync_file_path, 'w') as f:
                for time_series in self.camera_color_sync_time_series[i]:
                    f.write(f"{time_series.time:.6f}{self.camera_color_exts[i]}\n")

        # 写入相机深度数据同步文件
        for i, name in enumerate(self.args.cameraDepthNames):
            sync_file_path = os.path.join(self.camera_depth_dirs[i], "sync.txt")
            os.makedirs(os.path.dirname(sync_file_path), exist_ok=True)
            with open(sync_file_path, 'w') as f:
                for time_series in self.camera_depth_sync_time_series[i]:
                    f.write(f"{time_series.time:.6f}{self.camera_depth_exts[i]}\n")

        # 写入机械臂关节状态同步文件
        for i, name in enumerate(self.args.armJointStateNames):
            sync_file_path = os.path.join(self.arm_joint_state_dirs[i], "sync.txt")
            os.makedirs(os.path.dirname(sync_file_path), exist_ok=True)
            with open(sync_file_path, 'w') as f:
                for time_series in self.arm_joint_state_sync_time_series[i]:
                    f.write(f"{time_series.time:.6f}{self.arm_joint_state_exts[i]}\n")

        # 写入定位姿态同步文件
        for i, name in enumerate(self.args.localizationPoseNames):
            sync_file_path = os.path.join(self.localization_pose_dirs[i], "sync.txt")
            os.makedirs(os.path.dirname(sync_file_path), exist_ok=True)
            with open(sync_file_path, 'w') as f:
                for time_series in self.localization_pose_sync_time_series[i]:
                    f.write(f"{time_series.time:.6f}{self.localization_pose_exts[i]}\n")

        # 写入相机点云数据同步文件
        for i, name in enumerate(self.args.cameraPointCloudNames):
            sync_file_path = os.path.join(self.camera_point_cloud_dirs[i], "sync.txt")
            os.makedirs(os.path.dirname(sync_file_path), exist_ok=True)
            with open(sync_file_path, 'w') as f:
                for time_series in self.camera_point_cloud_sync_time_series[i]:
                    f.write(f"{time_series.time:.6f}{self.camera_point_cloud_exts[i]}\n")

        # 写入机械臂末端位姿同步文件
        for i, name in enumerate(self.args.armEndPoseNames):
            sync_file_path = os.path.join(self.arm_end_pose_dirs[i], "sync.txt")
            os.makedirs(os.path.dirname(sync_file_path), exist_ok=True)
            with open(sync_file_path, 'w') as f:
                for time_series in self.arm_end_pose_sync_time_series[i]:
                    f.write(f"{time_series.time:.6f}{self.arm_end_pose_exts[i]}\n")

        # 写入夹爪编码器同步文件
        for i, name in enumerate(self.args.gripperEncoderNames):
            sync_file_path = os.path.join(self.gripper_encoder_dirs[i], "sync.txt")
            os.makedirs(os.path.dirname(sync_file_path), exist_ok=True)
            with open(sync_file_path, 'w') as f:
                for time_series in self.gripper_encoder_sync_time_series[i]:
                    f.write(f"{time_series.time:.6f}{self.gripper_encoder_exts[i]}\n")

        # 写入IMU 9轴数据同步文件
        for i, name in enumerate(self.args.imu9AxisNames):
            sync_file_path = os.path.join(self.imu_9axis_dirs[i], "sync.txt")
            os.makedirs(os.path.dirname(sync_file_path), exist_ok=True)
            with open(sync_file_path, 'w') as f:
                for time_series in self.imu_9axis_sync_time_series[i]:
                    f.write(f"{time_series.time:.6f}{self.imu_9axis_exts[i]}\n")

        # 写入激光雷达点云同步文件
        for i, name in enumerate(self.args.lidarPointCloudNames):
            sync_file_path = os.path.join(self.lidar_point_cloud_dirs[i], "sync.txt")
            os.makedirs(os.path.dirname(sync_file_path), exist_ok=True)
            with open(sync_file_path, 'w') as f:
                for time_series in self.lidar_point_cloud_sync_time_series[i]:
                    f.write(f"{time_series.time:.6f}{self.lidar_point_cloud_exts[i]}\n")

        # 写入机器人底盘速度同步文件
        for i, name in enumerate(self.args.robotBaseVelNames):
            sync_file_path = os.path.join(self.robot_base_vel_dirs[i], "sync.txt")
            os.makedirs(os.path.dirname(sync_file_path), exist_ok=True)
            with open(sync_file_path, 'w') as f:
                for time_series in self.robot_base_vel_sync_time_series[i]:
                    f.write(f"{time_series.time:.6f}{self.robot_base_vel_exts[i]}\n")

        # 写入升降电机同步文件
        for i, name in enumerate(self.args.liftMotorNames):
            sync_file_path = os.path.join(self.lift_motor_dirs[i], "sync.txt")
            os.makedirs(os.path.dirname(sync_file_path), exist_ok=True)
            with open(sync_file_path, 'w') as f:
                for time_series in self.lift_motor_sync_time_series[i]:
                    f.write(f"{time_series.time:.6f}{self.lift_motor_exts[i]}\n")

    def process(self):
        self.load_all_time_series()
        self.sync()

def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--datasetDir', action='store', type=str, help='datasetDir',
                       default='/home/agilex/data', required=False)
    parser.add_argument('--episodeName', action='store', type=str, help='episodeName',
                       default="", required=False)
    parser.add_argument('--timeDiffLimit', action='store', type=float, help='timeDiffLimit',
                       default=0.03, required=False)
    parser.add_argument('--type', action='store', type=str, help='type',
                       default='aloha', required=False)
    parser.add_argument('--cameraColorNames', action='store', type=str, help='cameraColorNames',
                       default=[], required=False)
    parser.add_argument('--cameraDepthNames', action='store', type=str, help='cameraDepthNames',
                       default=[], required=False)
    parser.add_argument('--cameraPointCloudNames', action='store', type=str, help='cameraPointCloudNames',
                       default=[], required=False)
    parser.add_argument('--armJointStateNames', action='store', type=str, help='armJointStateNames',
                       default=[], required=False)
    parser.add_argument('--armEndPoseNames', action='store', type=str, help='armEndPoseNames',
                       default=[], required=False)
    parser.add_argument('--localizationPoseNames', action='store', type=str, help='localizationPoseNames',
                       default=[], required=False)
    parser.add_argument('--gripperEncoderNames', action='store', type=str, help='gripperEncoderNames',
                       default=[], required=False)
    parser.add_argument('--imu9AxisNames', action='store', type=str, help='imu9AxisNames',
                       default=[], required=False)
    parser.add_argument('--lidarPointCloudNames', action='store', type=str, help='lidarPointCloudNames',
                       default=[], required=False)
    parser.add_argument('--robotBaseVelNames', action='store', type=str, help='robotBaseVelNames',
                       default=[], required=False)
    parser.add_argument('--liftMotorNames', action='store', type=str, help='liftMotorNames',
                       default=[], required=False)
    args = parser.parse_args()

    with open(f'../install/data_tools/share/data_tools/config/{args.type}_data_params.yaml', 'r') as file:
        yaml_data = yaml.safe_load(file)
        args.cameraColorNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('camera', {}).get('color', {}).get('names', [])
        args.cameraDepthNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('camera', {}).get('depth', {}).get('names', [])
        args.cameraPointCloudNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('camera', {}).get('pointCloud', {}).get('names', [])
        args.armJointStateNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('arm', {}).get('jointState', {}).get('names', [])
        args.armEndPoseNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('arm', {}).get('endPose', {}).get('names', [])
        args.localizationPoseNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('localization', {}).get('pose', {}).get('names', [])
        args.gripperEncoderNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('gripper', {}).get('encoder', {}).get('names', [])
        args.imu9AxisNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('imu', {}).get('9axis', {}).get('names', [])
        args.lidarPointCloudNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('lidar', {}).get('pointCloud', {}).get('names', [])
        args.robotBaseVelNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('robotBase', {}).get('vel', {}).get('names', [])
        args.liftMotorNames = yaml_data.get('/**', {}).get('ros__parameters', {}).get('dataInfo', {}).get('lift', {}).get('motor', {}).get('names', [])
    return args


def main():
    args = get_arguments()
    if args.episodeName == "":
        if not os.path.exists(args.datasetDir):
            print(f"Error: Dataset directory {args.datasetDir} does not exist")
            return
        for f in os.listdir(args.datasetDir):
            if not f.endswith(".tar.gz"):
                args.episodeName = f
                print("episode name:", args.episodeName, "processing")
                operator = Operator(args)
                operator.process()
                print("episode name:", args.episodeName, "done")
    else:
        operator = Operator(args)
        operator.process()
    
    print("Done")


if __name__ == '__main__':
    main() 