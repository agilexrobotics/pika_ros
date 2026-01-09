#!/home/lin/miniconda3/envs/aloha/bin/python
# -- coding: UTF-8
"""
#!/root/miniconda3/envs/aloha/bin/python
#!/home/lin/miniconda3/envs/aloha/bin/python
"""

import os
import numpy as np
import argparse
import pcl
import struct
import open3d as o3d
import time as systime
import random
import cv2
import json
import numpy as np
import cv2
from tqdm import tqdm


def create_transformation_matrix(x, y, z, roll, pitch, yaw):
    transformation_matrix = np.eye(4, dtype=np.float64)
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


def depth_to_color_projection(depth_image, color_intrinsic, depth_intrinsic, extrinsic):
    # 获取深度图像的宽度和高度
    depth_height, depth_width = depth_image.shape[:2]

    # 创建网格坐标
    u, v = np.meshgrid(np.arange(depth_width), np.arange(depth_height))
    u = u.flatten()
    v = v.flatten()
    depth_values = depth_image.flatten()

    # 将像素坐标转换为齐次坐标
    depth_points = np.vstack((u, v, np.ones_like(u)))

    # 将深度图像中的点转换到深度相机坐标系
    X_depth = np.linalg.inv(depth_intrinsic) @ depth_points

    # 将深度相机坐标系中的点转换到彩色相机坐标系
    X_color = extrinsic @ np.vstack((X_depth, np.ones((1, X_depth.shape[1]))))

    # 将彩色相机坐标系中的点投影到彩色图像平面
    x_color = (color_intrinsic[0, 0] * (X_color[0, :] / X_color[2, :]) + color_intrinsic[0, 2]).round().astype(int)
    y_color = (color_intrinsic[1, 1] * (X_color[1, :] / X_color[2, :]) + color_intrinsic[1, 2]).round().astype(int)

    # 创建对齐后的深度图像
    aligned_depth = np.zeros_like(depth_image)

    # 将投影后的点存储到对齐后的深度图像中
    valid_indices = (x_color >= 0) & (x_color < depth_image.shape[1]) & (y_color >= 0) & (y_color < depth_image.shape[0])
    aligned_depth[y_color[valid_indices], x_color[valid_indices]] = depth_values[valid_indices]

    return aligned_depth


def color_depth_to_point_cloud(color_image_path, depth_image_path, color_intrinsic, depth_intrinsic, color_extrinsic, depth_extrinsic):
    # 读取 color 图像
    color_image = cv2.imread(color_image_path)
    if color_image is None:
        raise FileNotFoundError(f"color image {color_image_path} not found")

    # 读取深度图像
    depth_image = cv2.imread(depth_image_path, cv2.IMREAD_ANYDEPTH)
    if depth_image is None:
        raise FileNotFoundError(f"Depth image {depth_image_path} not found")
    if not np.array_equal(color_extrinsic, depth_extrinsic):
        depth_image = depth_to_color_projection(depth_image, color_intrinsic, depth_intrinsic, np.dot(np.linalg.inv(color_extrinsic), depth_extrinsic))
        # 相机内参矩阵
        fx, fy = color_intrinsic[0][0], color_intrinsic[1][1]
        cx, cy = color_intrinsic[0][2], color_intrinsic[1][2]
    else:
        # 相机内参矩阵
        fx, fy = depth_intrinsic[0][0], depth_intrinsic[1][1]
        cx, cy = depth_intrinsic[0][2], depth_intrinsic[1][2]
    # 获取图像的宽度和高度
    height, width = depth_image.shape

    u, v = np.meshgrid(np.arange(width), np.arange(height))
    u = u.astype(np.float32)
    v = v.astype(np.float32)
    z = depth_image.astype(np.float32) / 1000.0  # 将深度图像转换为米

    # 计算 3D 坐标
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    # 提取 color 颜色值
    b = color_image[..., 0].astype(np.float32)
    g = color_image[..., 1].astype(np.float32)
    r = color_image[..., 2].astype(np.float32)

    # 合并为点云
    point_cloud = np.stack((x, y, z, r, g, b), axis=-1)

    # 跳过深度为零的点
    valid_mask = z > 0.0
    point_cloud = point_cloud[valid_mask]

    return point_cloud


class Operator:
    def __init__(self, args):
        self.args = args
        self.episodeDir = os.path.join(self.args.datasetDir, "episode" + str(self.args.episodeIndex))

        self.cameraColorDirs = [os.path.join(self.episodeDir, "camera/color/" + self.args.cameraNames[i]) for i in range(len(self.args.cameraNames))]
        self.cameraColorConfigDirs = [os.path.join(self.cameraColorDirs[i], "config.json") for i in range(len(self.args.cameraNames))]
        self.cameraColorSyncDirs = [os.path.join(self.cameraColorDirs[i], "sync.txt") for i in range(len(self.args.cameraNames))]

        self.cameraDepthDirs = [os.path.join(self.episodeDir, "camera/depth/" + self.args.cameraNames[i]) for i in range(len(self.args.cameraNames))]
        self.cameraDepthConfigDirs = [os.path.join(self.cameraDepthDirs[i], "config.json") for i in range(len(self.args.cameraNames))]
        self.cameraDepthSyncDirs = [os.path.join(self.cameraDepthDirs[i], "sync.txt") for i in range(len(self.args.cameraNames))]

        self.cameraPointCloudDirs = [os.path.join(self.episodeDir, "camera/pointCloud/" + self.args.cameraNames[i]) for i in range(len(self.args.cameraNames))]
        self.cameraPointCloudConfigDirs = [os.path.join(self.cameraPointCloudDirs[i], "config.json") for i in range(len(self.args.cameraNames))]
        self.cameraPointCloudSyncDirs = [os.path.join(self.cameraPointCloudDirs[i], "sync.txt") for i in range(len(self.args.cameraNames))]

    def process(self):
        try:
            for i in range(len(self.args.cameraNames)):
                for f in tqdm(os.listdir(self.cameraColorDirs[i]), desc='color'):
                    if f.endswith(".png"):
                        source_path = os.path.join(self.cameraColorDirs[i], f)
                        target_path = os.path.join(self.cameraColorDirs[i], f[:-4]+".jpg")
                        color_image = cv2.imread(source_path)
                        if color_image is not None:
                            cv2.imwrite(target_path, color_image)
                        os.system(f"rm -rf {source_path}")
                with open(self.cameraColorConfigDirs[i], 'r') as color_config_file:
                    data = json.load(color_config_file)
                    color_intrinsic = np.array(data["K"]).reshape(3, 3)
                    color_extrinsic = create_transformation_matrix(data["parent_frame"]['x'], data["parent_frame"]['y'],
                                                                   data["parent_frame"]['z'], data["parent_frame"]['roll'],
                                                                   data["parent_frame"]['pitch'], data["parent_frame"]['yaw'])
                    with open(self.cameraDepthConfigDirs[i], 'r') as depth_config_file:
                        data = json.load(depth_config_file)
                        depth_intrinsic = np.array(data["K"]).reshape(3, 3)
                        depth_extrinsic = create_transformation_matrix(data["parent_frame"]['x'], data["parent_frame"]['y'],
                                                                       data["parent_frame"]['z'], data["parent_frame"]['roll'],
                                                                       data["parent_frame"]['pitch'], data["parent_frame"]['yaw'])
                        for f in tqdm(os.listdir(self.cameraDepthDirs[i]), desc='depth'):
                            if f.endswith(".png"):
                                depth_path = os.path.join(self.cameraDepthDirs[i], f)
                                depth_image = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
                                if depth_image is None:
                                    os.system(f"rm -rf {depth_path}")
                                elif not np.array_equal(color_extrinsic, depth_extrinsic):
                                    depth_image = depth_to_color_projection(depth_image, color_intrinsic, depth_intrinsic,
                                                                            np.dot(np.linalg.inv(color_extrinsic), depth_extrinsic))
                                    cv2.imwrite(depth_path, depth_image)
                        os.system(f"cp -r {self.cameraColorConfigDirs[i]} {self.cameraDepthConfigDirs[i]}")
                os.system(f"rm -rf {self.cameraPointCloudDirs[i]}")
        except:
            return

def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--datasetDir', action='store', type=str, help='datasetDir',
                        default="/home/agilex/data", required=False)
    parser.add_argument('--episodeIndex', action='store', type=int, help='episodeIndex',
                        default=-1, required=False)
    parser.add_argument('--cameraNames', action='store', type=str, help='cameraNames',
                        default=['left', 'front', 'right'], required=False)
    args = parser.parse_args()
    return args


def modify(args, dataset_dir):
    for f in os.listdir(dataset_dir):
        if f.endswith(".tar.gz"):
            print(os.path.join(dataset_dir, f))
            args.datasetDir = "/home/agilex"
            os.system(f"tar -zxf {os.path.join(dataset_dir, f)} -C {args.datasetDir}")
            try:
                args.episodeIndex = int(f[7:-7])
            except ValueError:
                continue
            print("episode index ", args.episodeIndex, "processing")
            operator = Operator(args)
            operator.process()
            os.system(f"bash -c 'source ~/pika_ws/install/setup.bash' && ros2 launch data_tools run_aloha_data_sync.launch.py datasetDir:={args.datasetDir} episodeIndex:={args.episodeIndex}")
            os.system(f"cd {args.datasetDir} && tar -zcf episode{args.episodeIndex}.tar.gz episode{args.episodeIndex} && rm -rf episode{args.episodeIndex}")
            os.system(f"rm -rf {dataset_dir}/episode{args.episodeIndex}.tar.gz")
            os.system(f"cp -r {args.datasetDir}/episode{args.episodeIndex}.tar.gz {dataset_dir}/episode{args.episodeIndex}.tar.gz")
            os.system(f"rm -rf {args.datasetDir}/episode{args.episodeIndex}.tar.gz")
            print("episode index ", args.episodeIndex, "done")
        else:
            modify(args, os.path.join(dataset_dir, f))


def main():
    args = get_arguments()
    dataset_dir = args.datasetDir
    if args.episodeIndex == -1:
        modify(args, dataset_dir)
    else:
        f = f"episode{args.episodeIndex}.tar.gz"
        print(os.path.join(dataset_dir, f))
        args.datasetDir = "/home/agilex"
        os.system(f"tar -zxf {os.path.join(dataset_dir, f)} -C {args.datasetDir}")
        args.episodeIndex = int(f[7:-7])
        print("episode index ", args.episodeIndex, "processing")
        operator = Operator(args)
        operator.process()
        os.system(f"bash -c 'source ~/pika_ws/install/setup.bash' && ros2 launch data_tools run_aloha_data_sync.launch.py datasetDir:={args.datasetDir} episodeIndex:={args.episodeIndex}")
        os.system(f"cd {args.datasetDir} && tar -zcf episode{args.episodeIndex}.tar.gz episode{args.episodeIndex} && rm -rf episode{args.episodeIndex}")
        os.system(f"rm -rf {dataset_dir}/episode{args.episodeIndex}.tar.gz")
        os.system(f"cp -r {args.datasetDir}/episode{args.episodeIndex}.tar.gz {dataset_dir}/episode{args.episodeIndex}.tar.gz")
        os.system(f"rm -rf {args.datasetDir}/episode{args.episodeIndex}.tar.gz")
        print("episode index ", args.episodeIndex, "done")
    print("Done")


if __name__ == '__main__':
    main()

