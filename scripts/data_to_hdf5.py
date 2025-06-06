#!/home/lin/miniconda3/envs/aloha/bin/python
# -- coding: UTF-8
"""
#!/root/miniconda3/envs/aloha/bin/python
#!/home/lin/miniconda3/envs/aloha/bin/python
"""
import math
import os
import numpy as np
import h5py
import argparse
import json
import cv2
from scipy.spatial.transform import Rotation as R
import yaml


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


class Operator:
    def __init__(self, args):
        self.args = args
        self.episodeDir = os.path.join(self.args.datasetDir, "episode" + str(self.args.episodeIndex))
        self.cameraColorDirs = [os.path.join(self.episodeDir, "camera/color/" + self.args.cameraColorNames[i]) for i in range(len(self.args.cameraColorNames))]
        self.cameraDepthDirs = [os.path.join(self.episodeDir, "camera/depth/" + self.args.cameraDepthNames[i]) for i in range(len(self.args.cameraDepthNames))]
        self.cameraPointCloudDirs = [os.path.join(self.episodeDir, "camera/pointCloud/" + self.args.cameraPointCloudNames[i] + ("-normalization" if self.args.useCameraPointCloudNormalization else "")) for i in range(len(self.args.cameraPointCloudNames))]
        self.armJointStateDirs = [os.path.join(self.episodeDir, "arm/jointState/" + self.args.armJointStateNames[i]) for i in range(len(self.args.armJointStateNames))]
        self.armEndPoseDirs = [os.path.join(self.episodeDir, "arm/endPose/" + self.args.armEndPoseNames[i]) for i in range(len(self.args.armEndPoseNames))]
        self.localizationPoseDirs = [os.path.join(self.episodeDir, "localization/pose/" + self.args.localizationPoseNames[i]) for i in range(len(self.args.localizationPoseNames))]
        self.gripperEncoderDirs = [os.path.join(self.episodeDir, "gripper/encoder/" + self.args.gripperEncoderNames[i]) for i in range(len(self.args.gripperEncoderNames))]
        self.imu9AxisDirs = [os.path.join(self.episodeDir, "imu/9axis/" + self.args.imu9AxisNames[i]) for i in range(len(self.args.imu9AxisNames))]
        self.lidarPointCloudDirs = [os.path.join(self.episodeDir, "lidar/pointCloud/" + self.args.lidarPointCloudNames[i]) for i in range(len(self.args.lidarPointCloudNames))]
        self.robotBaseVelDirs = [os.path.join(self.episodeDir, "robotBase/vel/" + self.args.robotBaseVelNames[i]) for i in range(len(self.args.robotBaseVelNames))]
        self.liftMotorDirs = [os.path.join(self.episodeDir, "lift/motor/" + self.args.liftMotorNames[i]) for i in range(len(self.args.liftMotorNames))]
        
        self.cameraColorSyncDirs = [os.path.join(self.cameraColorDirs[i], "sync.txt") for i in range(len(self.args.cameraColorNames))]
        self.cameraDepthSyncDirs = [os.path.join(self.cameraDepthDirs[i], "sync.txt") for i in range(len(self.args.cameraDepthNames))]
        self.cameraPointCloudSyncDirs = [os.path.join(self.cameraPointCloudDirs[i], "sync.txt") for i in range(len(self.args.cameraPointCloudNames))]
        self.armJointStateSyncDirs = [os.path.join(self.armJointStateDirs[i], "sync.txt") for i in range(len(self.args.armJointStateNames))]
        self.armEndPoseSyncDirs = [os.path.join(self.armEndPoseDirs[i], "sync.txt") for i in range(len(self.args.armEndPoseNames))]
        self.localizationPoseSyncDirs = [os.path.join(self.localizationPoseDirs[i], "sync.txt") for i in range(len(self.args.localizationPoseNames))]
        self.gripperEncoderSyncDirs = [os.path.join(self.gripperEncoderDirs[i], "sync.txt") for i in range(len(self.args.gripperEncoderNames))]
        self.imu9AxisSyncDirs = [os.path.join(self.imu9AxisDirs[i], "sync.txt") for i in range(len(self.args.imu9AxisNames))]
        self.lidarPointCloudSyncDirs = [os.path.join(self.lidarPointCloudDirs[i], "sync.txt") for i in range(len(self.args.lidarPointCloudNames))]
        self.robotBaseVelSyncDirs = [os.path.join(self.robotBaseVelDirs[i], "sync.txt") for i in range(len(self.args.robotBaseVelNames))]
        self.liftMotorSyncDirs = [os.path.join(self.liftMotorDirs[i], "sync.txt") for i in range(len(self.args.liftMotorNames))]

        self.cameraColorConfigDirs = [os.path.join(self.cameraColorDirs[i], "config.json") for i in range(len(self.args.cameraColorNames))]
        self.cameraDepthConfigDirs = [os.path.join(self.cameraDepthDirs[i], "config.json") for i in range(len(self.args.cameraDepthNames))]
        self.cameraPointCloudConfigDirs = [os.path.join(self.cameraPointCloudDirs[i], "config.json") for i in range(len(self.args.cameraPointCloudNames))]

        self.instructionsDir = os.path.join(self.episodeDir, "instructions.npy")
        if self.args.useIndex:
            self.dataFile = os.path.join(self.episodeDir, "data.hdf5")
        else:
            self.dataFile = os.path.join(self.args.datasetTargetDir, f"episode{self.args.episodeIndex}.hdf5")

    def process(self):
        data_dict = {}
        for cameraColorName in self.args.cameraColorNames:
            data_dict[f'camera/color/{cameraColorName}'] = []
            data_dict[f'camera/colorIntrinsic/{cameraColorName}'] = []
            data_dict[f'camera/colorExtrinsic/{cameraColorName}'] = []
        for cameraDepthName in self.args.cameraDepthNames:
            data_dict[f'camera/depth/{cameraDepthName}'] = []
            data_dict[f'camera/depthIntrinsic/{cameraDepthName}'] = []
            data_dict[f'camera/depthExtrinsic/{cameraDepthName}'] = []
        for cameraPointCloudName in self.args.cameraPointCloudNames:
            data_dict[f'camera/pointCloud/{cameraPointCloudName}'] = []
            data_dict[f'camera/pointCloudIntrinsic/{cameraPointCloudName}'] = []
            data_dict[f'camera/pointCloudExtrinsic/{cameraPointCloudName}'] = []
        for armJointStateName in self.args.armJointStateNames:
            data_dict[f'arm/jointStateVelocity/{armJointStateName}'] = []
            data_dict[f'arm/jointStatePosition/{armJointStateName}'] = []
            data_dict[f'arm/jointStateEffort/{armJointStateName}'] = []
        for armEndPoseName in self.args.armEndPoseNames:
            data_dict[f'arm/endPose/{armEndPoseName}'] = []
        for localizationPoseName in self.args.localizationPoseNames:
            data_dict[f'localization/pose/{localizationPoseName}'] = []
        for gripperEncoderName in self.args.gripperEncoderNames:
            data_dict[f'gripper/encoderAngle/{gripperEncoderName}'] = []
            data_dict[f'gripper/encoderDistance/{gripperEncoderName}'] = []
        for imu9AxisName in self.args.imu9AxisNames:
            data_dict[f'imu/9axisOrientation/{imu9AxisName}'] = []
            data_dict[f'imu/9axisAngularVelocity/{imu9AxisName}'] = []
            data_dict[f'imu/9axisLinearAcceleration/{imu9AxisName}'] = []
        for lidarPointCloudName in self.args.lidarPointCloudNames:
            data_dict[f'lidar/pointCloud/{lidarPointCloudName}'] = []
        for robotBaseVelName in self.args.robotBaseVelNames:
            data_dict[f'robotBase/vel/{robotBaseVelName}'] = []
        for liftMotorName in self.args.liftMotorNames:
            data_dict[f'lift/motor/{liftMotorName}'] = []
        data_dict[f'instruction'] = self.instructionsDir
        data_dict[f'timestamp'] = []
        data_dict[f'size'] = 0
        size_count = 0
        for i in range(len(self.args.cameraColorNames)):
            with open(self.cameraColorSyncDirs[i], 'r') as lines:
                count = 0
                for line in lines:
                    line = line.replace('\n', '')
                    time = float(line[:line.rfind(".")])
                    if len(data_dict[f'timestamp']) <= count:
                        data_dict[f'timestamp'].append(time)
                    else:
                        data_dict[f'timestamp'][count] = time if time < data_dict[f'timestamp'][count] else data_dict[f'timestamp'][count]
                    if self.args.useIndex:
                        data_dict[f'camera/color/{self.args.cameraColorNames[i]}'].append(os.path.join(self.cameraColorDirs[i][len(self.episodeDir)+1:], line))
                    else:
                        data_dict[f'camera/color/{self.args.cameraColorNames[i]}'].append(cv2.imread(os.path.join(self.cameraColorDirs[i], line)))
                    # print(os.path.join(self.cameraColorDirs[i], line))
                    # cv2.imread(os.path.join(self.cameraColorDirs[i], line))
                    count += 1
                if size_count == 0:
                    size_count = count
            with open(self.cameraColorConfigDirs[i], 'r') as color_config_file:
                data = json.load(color_config_file)
                color_intrinsic = np.array(data["K"]).reshape(3, 3)
                color_extrinsic = create_transformation_matrix(data["parent_frame"]['x'],  data["parent_frame"]['y'], data["parent_frame"]['z'], data["parent_frame"]['roll'], data["parent_frame"]['pitch'], data["parent_frame"]['yaw'])
                data_dict[f'camera/colorIntrinsic/{self.args.cameraColorNames[i]}'] = color_intrinsic
                data_dict[f'camera/colorExtrinsic/{self.args.cameraColorNames[i]}'] = color_extrinsic
        for i in range(len(self.args.cameraDepthNames)):
            with open(self.cameraDepthSyncDirs[i], 'r') as lines:
                count = 0
                for line in lines:
                    line = line.replace('\n', '')
                    time = float(line[:line.rfind(".")])
                    if len(data_dict[f'timestamp']) <= count:
                        data_dict[f'timestamp'].append(time)
                    else:
                        data_dict[f'timestamp'][count] = time if time < data_dict[f'timestamp'][count] else data_dict[f'timestamp'][count]
                    if self.args.useIndex:
                        data_dict[f'camera/depth/{self.args.cameraDepthNames[i]}'].append(os.path.join(self.cameraDepthDirs[i][len(self.episodeDir)+1:], line))
                    else:
                        data_dict[f'camera/depth/{self.args.cameraDepthNames[i]}'].append(cv2.imread(os.path.join(self.cameraDepthDirs[i], line)))
                    # print(os.path.join(self.cameraDepthDirs[i], line))
                    # img = cv2.imread(os.path.join(self.cameraDepthDirs[i], line), cv2.IMREAD_UNCHANGED).flatten()
                    # print(max(img), min(img))
                    count += 1
                if size_count == 0:
                    size_count = count
            with open(self.cameraDepthConfigDirs[i], 'r') as depth_config_file:
                data = json.load(depth_config_file)
                depth_intrinsic = np.array(data["K"]).reshape(3, 3)
                depth_extrinsic = create_transformation_matrix(data["parent_frame"]['x'],  data["parent_frame"]['y'], data["parent_frame"]['z'], data["parent_frame"]['roll'], data["parent_frame"]['pitch'], data["parent_frame"]['yaw'])
                data_dict[f'camera/depthIntrinsic/{self.args.cameraDepthNames[i]}'] = depth_intrinsic
                data_dict[f'camera/depthExtrinsic/{self.args.cameraDepthNames[i]}'] = depth_extrinsic
        for i in range(len(self.args.cameraPointCloudNames)):
            with open(self.cameraPointCloudSyncDirs[i], 'r') as lines:
                count = 0
                for line in lines:
                    line = line.replace('\n', '')
                    time = float(line[:line.rfind(".")])
                    if len(data_dict[f'timestamp']) <= count:
                        data_dict[f'timestamp'].append(time)
                    else:
                        data_dict[f'timestamp'][count] = time if time < data_dict[f'timestamp'][count] else data_dict[f'timestamp'][count]
                    if self.args.useIndex:
                        data_dict[f'camera/pointCloud/{self.args.cameraPointCloudNames[i]}'].append(os.path.join(self.cameraPointCloudDirs[i][len(self.episodeDir)+1:], line))
                    else:
                        data_dict[f'camera/pointCloud/{self.args.cameraPointCloudNames[i]}'].append(np.load(os.path.join(self.cameraPointCloudDirs[i], line)))
                    count += 1
                if size_count == 0:
                    size_count = count
            with open(self.cameraPointCloudConfigDirs[i], 'r') as point_cloud_config_file:
                data = json.load(point_cloud_config_file)
                point_cloud_intrinsic = np.array(data["K"]).reshape(3, 3)
                point_cloud_extrinsic = create_transformation_matrix(data["parent_frame"]['x'], data["parent_frame"]['y'], data["parent_frame"]['z'], data["parent_frame"]['roll'], data["parent_frame"]['pitch'], data["parent_frame"]['yaw'])
                data_dict[f'camera/pointCloudIntrinsic/{self.args.cameraPointCloudNames[i]}'] = point_cloud_intrinsic
                data_dict[f'camera/pointCloudExtrinsic/{self.args.cameraPointCloudNames[i]}'] = point_cloud_extrinsic
        for i in range(len(self.args.armJointStateNames)):
            with open(self.armJointStateSyncDirs[i], 'r') as lines:
                count = 0
                for line in lines:
                    line = line.replace('\n', '')
                    time = float(line[:line.rfind(".")])
                    if len(data_dict[f'timestamp']) <= count:
                        data_dict[f'timestamp'].append(time)
                    else:
                        data_dict[f'timestamp'][count] = time if time < data_dict[f'timestamp'][count] else data_dict[f'timestamp'][count]
                    with open(os.path.join(self.armJointStateDirs[i], line), 'r') as file:
                        data = json.load(file)
                        data_dict[f'arm/jointStateVelocity/{self.args.armJointStateNames[i]}'].append(np.array(data['velocity']))
                        data_dict[f'arm/jointStateEffort/{self.args.armJointStateNames[i]}'].append(np.array(data['effort']))
                        data_dict[f'arm/jointStatePosition/{self.args.armJointStateNames[i]}'].append(np.array(data['position']))
                    count += 1
                if size_count == 0:
                    size_count = count
        for i in range(len(self.args.armEndPoseNames)):
            with open(self.armEndPoseSyncDirs[i], 'r') as lines:
                count = 0
                for line in lines:
                    line = line.replace('\n', '')
                    time = float(line[:line.rfind(".")])
                    if len(data_dict[f'timestamp']) <= count:
                        data_dict[f'timestamp'].append(time)
                    else:
                        data_dict[f'timestamp'][count] = time if time < data_dict[f'timestamp'][count] else data_dict[f'timestamp'][count]
                    with open(os.path.join(self.armEndPoseDirs[i], line), 'r') as file:
                        data = json.load(file)
                        if 'grasper' in data.keys():
                            data_dict[f'arm/endPose/{self.args.armEndPoseNames[i]}'].append(np.array([data['x'], data['y'], data['z'], data['roll'], data['pitch'], data['yaw'], data['grasper']]))
                        else:
                            data_dict[f'arm/endPose/{self.args.armEndPoseNames[i]}'].append(np.array([data['x'], data['y'], data['z'], data['roll'], data['pitch'], data['yaw']]))
                    count += 1
                if size_count == 0:
                    size_count = count
        for i in range(len(self.args.localizationPoseNames)):
            with open(self.localizationPoseSyncDirs[i], 'r') as lines:
                count = 0
                for line in lines:
                    line = line.replace('\n', '')
                    time = float(line[:line.rfind(".")])
                    if len(data_dict[f'timestamp']) <= count:
                        data_dict[f'timestamp'].append(time)
                    else:
                        data_dict[f'timestamp'][count] = time if time < data_dict[f'timestamp'][count] else data_dict[f'timestamp'][count]
                    with open(os.path.join(self.localizationPoseDirs[i], line), 'r') as file:
                        data = json.load(file)
                        # ori_trans = create_transformation_matrix(data['x'], data['y'], data['z'], data['roll'], data['pitch'], data['yaw'])
                        # incre_trans = create_transformation_matrix(0, 0, 0, 0, math.pi/4, 0)
                        # final_trans = np.dot(ori_trans, incre_trans)
                        # xyzrpy = matrix_to_xyzrpy(final_trans)
                        data_dict[f'localization/pose/{self.args.localizationPoseNames[i]}'].append(np.array([data['x'], data['y'], data['z'], data['roll'], data['pitch'], data['yaw']]))
                    count += 1
                if size_count == 0:
                    size_count = count
        for i in range(len(self.args.gripperEncoderNames)):
            with open(self.gripperEncoderSyncDirs[i], 'r') as lines:
                count = 0
                for line in lines:
                    line = line.replace('\n', '')
                    time = float(line[:line.rfind(".")])
                    if len(data_dict[f'timestamp']) <= count:
                        data_dict[f'timestamp'].append(time)
                    else:
                        data_dict[f'timestamp'][count] = time if time < data_dict[f'timestamp'][count] else data_dict[f'timestamp'][count]
                    with open(os.path.join(self.gripperEncoderDirs[i], line), 'r') as file:
                        data = json.load(file)
                        data_dict[f'gripper/encoderAngle/{self.args.gripperEncoderNames[i]}'].append(data['angle'])
                        data_dict[f'gripper/encoderDistance/{self.args.gripperEncoderNames[i]}'].append(data['distance'])
                    count += 1
                if size_count == 0:
                    size_count = count
        for i in range(len(self.args.imu9AxisNames)):
            with open(self.imu9AxisSyncDirs[i], 'r') as lines:
                count = 0
                for line in lines:
                    line = line.replace('\n', '')
                    time = float(line[:line.rfind(".")])
                    if len(data_dict[f'timestamp']) <= count:
                        data_dict[f'timestamp'].append(time)
                    else:
                        data_dict[f'timestamp'][count] = time if time < data_dict[f'timestamp'][count] else data_dict[f'timestamp'][count]
                    with open(os.path.join(self.imu9AxisDirs[i], line), 'r') as file:
                        data = json.load(file)
                        data_dict[f'imu/9axisOrientation/{self.args.imu9AxisNames[i]}'].append(np.array([data['orientation']['x'], data['orientation']['y'], data['orientation']['z'], data['orientation']['w']]))
                        data_dict[f'imu/9axisAngularVelocity/{self.args.imu9AxisNames[i]}'].append(np.array([data['angular_velocity']['x'], data['angular_velocity']['y'], data['angular_velocity']['z']]))
                        data_dict[f'imu/9axisLinearAcceleration/{self.args.imu9AxisNames[i]}'].append(np.array([data['linear_acceleration']['x'], data['linear_acceleration']['y'], data['linear_acceleration']['z']]))
                    count += 1
                if size_count == 0:
                    size_count = count
        for i in range(len(self.args.lidarPointCloudNames)):
            with open(self.lidarPointCloudSyncDirs[i], 'r') as lines:
                count = 0
                for line in lines:
                    line = line.replace('\n', '')
                    time = float(line[:line.rfind(".")])
                    if len(data_dict[f'timestamp']) <= count:
                        data_dict[f'timestamp'].append(time)
                    else:
                        data_dict[f'timestamp'][count] = time if time < data_dict[f'timestamp'][count] else data_dict[f'timestamp'][count]
                    data_dict[f'lidar/pointCloud/{self.args.lidarPointCloudNames[i]}'].append(os.path.join(self.lidarPointCloudDirs[i][len(self.episodeDir)+1:], line))
                    count += 1
                if size_count == 0:
                    size_count = count
        for i in range(len(self.args.robotBaseVelNames)):
            with open(self.robotBaseVelSyncDirs[i], 'r') as lines:
                count = 0
                for line in lines:
                    line = line.replace('\n', '')
                    time = float(line[:line.rfind(".")])
                    if len(data_dict[f'timestamp']) <= count:
                        data_dict[f'timestamp'].append(time)
                    else:
                        data_dict[f'timestamp'][count] = time if time < data_dict[f'timestamp'][count] else data_dict[f'timestamp'][count]
                    with open(os.path.join(self.robotBaseVelDirs[i], line), 'r') as file:
                        data = json.load(file)
                        data_dict[f'robotBase/vel/{self.args.robotBaseVelNames[i]}'].append(np.array([data['linear']['x'], data['linear']['y'], data['angular']['z']]))
                    count += 1
                if size_count == 0:
                    size_count = count
        for i in range(len(self.args.liftMotorNames)):
            with open(self.liftMotorSyncDirs[i], 'r') as lines:
                count = 0
                for line in lines:
                    line = line.replace('\n', '')
                    time = float(line[:line.rfind(".")])
                    if len(data_dict[f'timestamp']) <= count:
                        data_dict[f'timestamp'].append(time)
                    else:
                        data_dict[f'timestamp'][count] = time if time < data_dict[f'timestamp'][count] else data_dict[f'timestamp'][count]
                    with open(os.path.join(self.liftMotorDirs[i], line), 'r') as file:
                        data = json.load(file)
                        data_dict[f'lift/motor/{self.args.liftMotorNames[i]}'].append(data['backHeight'])
                    count += 1
                if size_count == 0:
                    size_count = count
        data_dict['size'] = size_count
        with h5py.File(self.dataFile, 'w', rdcc_nbytes=1024 ** 2 * 2) as root:
            for key in data_dict:
                root.create_dataset(key, data=data_dict[key])


def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--datasetDir', action='store', type=str, help='datasetDir',
                        default="/home/agilex/data", required=False)
    parser.add_argument('--episodeIndex', action='store', type=int, help='episodeIndex',
                        default=-1, required=False)
    parser.add_argument('--datasetTargetDir', action='store', type=str, help='datasetTargetDir',
                        default="/home/agilex/data", required=False)
    parser.add_argument('--useIndex', action='store', type=bool, help='useIndex',
                        default=True, required=False)
    parser.add_argument('--type', action='store', type=str, help='type',
                        default="aloha", required=False)
    parser.add_argument('--cameraColorNames', action='store', type=str, help='cameraColorNames',
                        default=[], required=False)
    parser.add_argument('--cameraDepthNames', action='store', type=str, help='cameraDepthNames',
                        default=[], required=False)
    parser.add_argument('--cameraPointCloudNames', action='store', type=str, help='cameraPointCloudNames',
                        default=[], required=False)
    parser.add_argument('--useCameraPointCloud', action='store', type=bool, help='useCameraPointCloud',
                        default=False, required=False)
    parser.add_argument('--useCameraPointCloudNormalization', action='store', type=bool, help='useCameraPointCloudNormalization',
                        default=True, required=False)
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

    with open(f'../install/share/data_tools/config/{args.type}_data_params.yaml', 'r') as file:
        yaml_data = yaml.safe_load(file)
        args.cameraColorNames = yaml_data['dataInfo']['camera']['color']['names']
        args.cameraDepthNames = yaml_data['dataInfo']['camera']['depth']['names']
        args.cameraPointCloudNames = yaml_data['dataInfo']['camera']['pointCloud']['names']
        args.armJointStateNames = yaml_data['dataInfo']['arm']['jointState']['names']
        args.armEndPoseNames = yaml_data['dataInfo']['arm']['endPose']['names']
        args.localizationPoseNames = yaml_data['dataInfo']['localization']['pose']['names']
        args.gripperEncoderNames = yaml_data['dataInfo']['gripper']['encoder']['names']
        args.imu9AxisNames = yaml_data['dataInfo']['imu']['9axis']['names']
        args.lidarPointCloudNames = yaml_data['dataInfo']['lidar']['pointCloud']['names']
        args.robotBaseVelNames = yaml_data['dataInfo']['robotBase']['vel']['names']
        args.liftMotorNames = yaml_data['dataInfo']['lift']['motor']['names']

    return args


def main():
    args = get_arguments()
    if not args.useCameraPointCloud:
        args.cameraPointCloudNames = []
    if args.episodeIndex == -1:
        for f in os.listdir(args.datasetDir):
            if f.startswith("episode") and not f.endswith(".tar.gz"):
                args.episodeIndex = int(f[7:])
                print("episode index ", args.episodeIndex, "processing")
                operator = Operator(args)
                operator.process()
                print("episode index ", args.episodeIndex, "done")
    else:
        operator = Operator(args)
        operator.process()
    print("Done")


if __name__ == '__main__':
    main()
