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
import yaml


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
        self.episodeDir = os.path.join(self.args.datasetDir, self.args.episodeName)

        self.cameraColorDirs = [os.path.join(self.episodeDir, "camera/color/" + self.args.cameraNames[i]) for i in range(len(self.args.cameraNames))]
        self.cameraColorConfigDirs = [os.path.join(self.cameraColorDirs[i], "config.json") for i in range(len(self.args.cameraNames))]
        self.cameraColorSyncDirs = [os.path.join(self.cameraColorDirs[i], "sync.txt") for i in range(len(self.args.cameraNames))]

        self.cameraDepthDirs = [os.path.join(self.episodeDir, "camera/depth/" + self.args.cameraNames[i]) for i in range(len(self.args.cameraNames))]
        self.cameraDepthConfigDirs = [os.path.join(self.cameraDepthDirs[i], "config.json") for i in range(len(self.args.cameraNames))]
        self.cameraDepthSyncDirs = [os.path.join(self.cameraDepthDirs[i], "sync.txt") for i in range(len(self.args.cameraNames))]

        self.cameraPointCloudDirs = [os.path.join(self.episodeDir, "camera/pointCloud/" + self.args.cameraNames[i]) for i in range(len(self.args.cameraNames))]
        self.cameraPointCloudConfigDirs = [os.path.join(self.cameraPointCloudDirs[i], "config.json") for i in range(len(self.args.cameraNames))]
        self.cameraPointCloudSyncDirs = [os.path.join(self.cameraPointCloudDirs[i], "sync.txt") for i in range(len(self.args.cameraNames))]

        self.cameraPointCloudNormDirs = [os.path.join(self.episodeDir, "camera/pointCloud/" + self.args.cameraNames[i] + "-normalization") for i in range(len(self.args.cameraNames))]
        self.cameraPointCloudNormConfigDirs = [os.path.join(self.cameraPointCloudNormDirs[i], "config.json") for i in range(len(self.args.cameraNames))]
        self.cameraPointCloudNormSyncDirs = [os.path.join(self.cameraPointCloudNormDirs[i], "sync.txt") for i in range(len(self.args.cameraNames))]

    def farthest_point_sampling(self, points, k):
        sampled_points = [np.random.randint(len(points))]
        distances = np.linalg.norm(points - points[sampled_points[-1]], axis=1)
        for _ in range(k - 1):
            farthest_index = np.argmax(distances)
            sampled_points.append(farthest_index)
            distances = np.minimum(distances, np.linalg.norm(points - points[farthest_index], axis=1))
        return sampled_points

    def process(self):
        for i in range(len(self.args.cameraNames)):
            os.system(f"rm -rf {self.cameraPointCloudNormDirs[i]}")
            os.system(f"mkdir {self.cameraPointCloudNormDirs[i]}")
            use_point_cloud = os.path.exists(self.cameraPointCloudSyncDirs[i])
            if use_point_cloud:
                os.system(f"cp {self.cameraPointCloudConfigDirs[i]} {self.cameraPointCloudNormConfigDirs[i]}")
                with open(self.cameraPointCloudSyncDirs[i], 'r') as lines:
                    with open(self.cameraPointCloudNormSyncDirs[i], "w") as f:
                        for line in lines:
                            line = line.replace('\n', '')
                            time = line[:-4]
                            os.path.join(self.cameraPointCloudDirs[i], line)
                            print(os.path.join(self.cameraPointCloudDirs[i], line))
                            if self.args.voxelSize != 0:
                                pcd = o3d.io.read_point_cloud(os.path.join(self.cameraPointCloudDirs[i], line))

                                downsampled_cloud = pcd.voxel_down_sample(self.args.voxelSize)
                                if self.args.use_farthest_point_down_sample and len(
                                        downsampled_cloud.points) > self.args.pointNum:
                                    downsampled_cloud = downsampled_cloud.farthest_point_down_sample(self.args.pointNum)

                                downsampled_cloud.colors = o3d.utility.Vector3dVector(
                                    (np.asarray(downsampled_cloud.colors) * 255).astype(np.float64))
                                pc = np.concatenate([downsampled_cloud.points, downsampled_cloud.colors], axis=-1)
                                condition = pc[:, 2] < 2
                                pc = pc[condition, :]

                                if pc.shape[0] > self.args.pointNum:
                                    idxs = np.random.choice(pc.shape[0], self.args.pointNum, replace=False)
                                    pc = pc[idxs]
                                elif pc.shape[0] < self.args.pointNum:
                                    if pc.shape[0] == 0:
                                        pc = np.zeros([1, 4], dtype=np.float32)
                                    idxs1 = np.arange(pc.shape[0])
                                    idxs2 = np.random.choice(pc.shape[0], self.args.pointNum - pc.shape[0],
                                                             replace=True)
                                    idxs = np.concatenate([idxs1, idxs2], axis=0)
                                    pc = pc[idxs]
                            else:
                                pc = pcl.load_XYZRGB(os.path.join(self.cameraPointCloudDirs[i], line)).to_array()
                                condition = pc[:, 2] < 2
                                pc = pc[condition, :]
                                if pc.shape[0] >= self.args.pointNum:
                                    idxs = np.random.choice(pc.shape[0], self.args.pointNum, replace=False)
                                elif pc.shape[0] < self.args.pointNum:
                                    if pc.shape[0] == 0:
                                        pc = np.zeros([1, 4], dtype=np.float32)
                                    idxs1 = np.arange(pc.shape[0])
                                    idxs2 = np.random.choice(pc.shape[0], self.args.pointNum - pc.shape[0],
                                                             replace=True)
                                    idxs = np.concatenate([idxs1, idxs2], axis=0)

                                rgbs = pc[idxs][:, 3].view(np.uint32)
                                r = (np.right_shift(rgbs, 16) % 256)[:, np.newaxis]
                                g = (np.right_shift(rgbs, 8) % 256)[:, np.newaxis]
                                b = (rgbs % 256)[:, np.newaxis]
                                r_g_b = np.concatenate([r, g, b], axis=-1)
                                pc = np.concatenate([pc[idxs][:, :3], r_g_b], axis=-1)

                            with open(self.cameraColorConfigDirs[i], 'r') as color_config_file:
                                data = json.load(color_config_file)
                                color_extrinsic = create_transformation_matrix(data["parent_frame"]['x'], data["parent_frame"]['y'], data["parent_frame"]['z'], data["parent_frame"]['roll'], data["parent_frame"]['pitch'], data["parent_frame"]['yaw'])
                                with open(self.cameraPointCloudConfigDirs[i], 'r') as point_cloud_config_file:
                                    data = json.load(point_cloud_config_file)
                                    point_cloud_extrinsic = create_transformation_matrix(data["parent_frame"]['x'], data["parent_frame"]['y'], data["parent_frame"]['z'], data["parent_frame"]['roll'], data["parent_frame"]['pitch'], data["parent_frame"]['yaw'])
                                    if not np.array_equal(color_extrinsic, point_cloud_extrinsic):
                                        pcd = o3d.geometry.PointCloud()
                                        pcd.points = o3d.utility.Vector3dVector(pc[:, :3])
                                        pc[:, 3:] = pc[:, 3:] / 255
                                        pcd.colors = o3d.utility.Vector3dVector(pc[:, 3:])
                                        pcd.transform(np.dot(np.linalg.inv(color_extrinsic), point_cloud_extrinsic))
                                        # o3d.io.write_point_cloud(
                                        #     os.path.join(self.cameraPointCloudNormDirs[i], time + ".pcd"), pcd)
                                        pcd.colors = o3d.utility.Vector3dVector(
                                            (np.asarray(pcd.colors) * 255).astype(np.float64))
                                        pc = np.concatenate([pcd.points, pcd.colors], axis=-1)

                            if self.args.use_augment:
                                # t = random.randint(0, 10)
                                # for _ in range(t):
                                #     center_point_idx = random.randint(0, pc.shape[0] - 1)
                                #     dist = pc[center_point_idx, 2] / 2 * 0.20
                                #     width = random.random() * dist
                                #     height = random.random() * dist
                                #     condition = (np.array(pc[:, 0] > (pc[center_point_idx, 0] + width / 2)) | np.array(pc[:, 0] < (pc[center_point_idx, 0] - width / 2))) | \
                                #                 (np.array(pc[:, 1] > (pc[center_point_idx, 1] + height / 2)) | np.array(pc[:, 1] < (pc[center_point_idx, 1] - height / 2)))
                                #     condition = np.logical_not(condition)
                                #     replace = np.random.uniform(-2, 2, 3)
                                #     replace[2] = np.random.uniform(0, 2)
                                #     pc[condition, :3] *= replace

                                t = random.randint(0, 200)
                                indexs = np.random.randint(0, pc.shape[0], t)
                                replace = np.random.uniform(0, 1, (t, 6))
                                replace[:, :2] = np.random.uniform(-1, 1, (t, 2))
                                replace[:, :3] *= 2
                                replace[:, 3:] *= 255
                                pc[indexs, :] = replace
                                # pc[indexs, :] *= np.random.uniform(0, 1, (t, 6))

                                t = random.randint(0, 1000)
                                indexs = np.random.randint(0, pc.shape[0], t)
                                replace = np.random.uniform(0, 2, (t, 3))
                                replace[:, 1] = replace[:, 0]
                                replace[:, 2] = replace[:, 0]
                                pc[indexs, 3:] *= replace
                                pc[indexs, 3:] = np.clip(pc[indexs, 3:], 0, 255)

                                t = random.randint(0, 200)
                                indexs = np.random.randint(0, pc.shape[0], t)
                                pc[indexs, 3:] = np.random.uniform(0, 1, (t, 3)) * 255
                                # pc[indexs, 3:] *= np.random.uniform(0, 1, (t, 3))

                            f.write(time + ".npy\n")
                            np.save(os.path.join(self.cameraPointCloudNormDirs[i], time + ".npy"), pc)
            else:
                os.system(f"cp {self.cameraDepthConfigDirs[i]} {self.cameraPointCloudNormConfigDirs[i]}")
                with open(self.cameraColorConfigDirs[i], 'r') as color_config_file:
                    data = json.load(color_config_file)
                    color_intrinsic = np.array(data["K"]).reshape(3, 3)
                    color_extrinsic = create_transformation_matrix(data["parent_frame"]['x'], data["parent_frame"]['y'], data["parent_frame"]['z'], data["parent_frame"]['roll'], data["parent_frame"]['pitch'], data["parent_frame"]['yaw'])
                    with open(self.cameraDepthConfigDirs[i], 'r') as depth_config_file:
                        data = json.load(depth_config_file)
                        depth_intrinsic = np.array(data["K"]).reshape(3, 3)
                        depth_extrinsic = create_transformation_matrix(data["parent_frame"]['x'], data["parent_frame"]['y'], data["parent_frame"]['z'], data["parent_frame"]['roll'], data["parent_frame"]['pitch'], data["parent_frame"]['yaw'])
                        with open(self.cameraColorSyncDirs[i], 'r') as color_lines:
                            with open(self.cameraDepthSyncDirs[i], 'r') as depth_lines:
                                with open(self.cameraPointCloudNormSyncDirs[i], "w") as f:
                                    for color_line, depth_line in zip(color_lines, depth_lines):
                                        color_line = color_line.rstrip()
                                        depth_line = depth_line.rstrip()
                                        time = depth_line[:-4]
                                        print(os.path.join(self.cameraDepthDirs[i], depth_line))
                                        point_cloud = color_depth_to_point_cloud(os.path.join(self.cameraColorDirs[i], color_line), os.path.join(self.cameraDepthDirs[i], depth_line), color_intrinsic, depth_intrinsic, color_extrinsic, depth_extrinsic)
                                        if self.args.voxelSize != 0:
                                            pcd = o3d.geometry.PointCloud()
                                            pcd.points = o3d.utility.Vector3dVector(point_cloud[:, :3])
                                            pcd.colors = o3d.utility.Vector3dVector(point_cloud[:, 3:] / 255.0)
                                            # o3d.io.write_point_cloud(
                                            #     os.path.join(self.cameraPointCloudNormDirs[i], time + ".pcd"), pcd)
                                            downsampled_cloud = pcd.voxel_down_sample(self.args.voxelSize)
                                            if self.args.use_farthest_point_down_sample and len(
                                                    downsampled_cloud.points) > self.args.pointNum:
                                                downsampled_cloud = downsampled_cloud.farthest_point_down_sample(self.args.pointNum)

                                            downsampled_cloud.colors = o3d.utility.Vector3dVector(
                                                (np.asarray(downsampled_cloud.colors) * 255).astype(np.float64))
                                            pc = np.concatenate([downsampled_cloud.points, downsampled_cloud.colors], axis=-1)
                                            condition = pc[:, 2] < 2
                                            pc = pc[condition, :]

                                            if pc.shape[0] > self.args.pointNum:
                                                idxs = np.random.choice(pc.shape[0], self.args.pointNum, replace=False)
                                                pc = pc[idxs]
                                            elif pc.shape[0] < self.args.pointNum:
                                                if pc.shape[0] == 0:
                                                    pc = np.zeros([1, 4], dtype=np.float32)
                                                idxs1 = np.arange(pc.shape[0])
                                                idxs2 = np.random.choice(pc.shape[0], self.args.pointNum - pc.shape[0],
                                                                         replace=True)
                                                idxs = np.concatenate([idxs1, idxs2], axis=0)
                                                pc = pc[idxs]
                                        else:
                                            pc = point_cloud
                                            condition = pc[:, 2] < 2
                                            pc = pc[condition, :]
                                            if pc.shape[0] >= self.args.pointNum:
                                                idxs = np.random.choice(pc.shape[0], self.args.pointNum, replace=False)
                                            elif pc.shape[0] < self.args.pointNum:
                                                if pc.shape[0] == 0:
                                                    pc = np.zeros([1, 4], dtype=np.float32)
                                                idxs1 = np.arange(pc.shape[0])
                                                idxs2 = np.random.choice(pc.shape[0], self.args.pointNum - pc.shape[0],
                                                                         replace=True)
                                                idxs = np.concatenate([idxs1, idxs2], axis=0)
                                            pc = pc[idxs]
                                        if self.args.use_augment:
                                            # t = random.randint(0, 10)
                                            # for _ in range(t):
                                            #     center_point_idx = random.randint(0, pc.shape[0] - 1)
                                            #     dist = pc[center_point_idx, 2] / 2 * 0.20
                                            #     width = random.random() * dist
                                            #     height = random.random() * dist
                                            #     condition = (np.array(pc[:, 0] > (pc[center_point_idx, 0] + width / 2)) | np.array(pc[:, 0] < (pc[center_point_idx, 0] - width / 2))) | \
                                            #                 (np.array(pc[:, 1] > (pc[center_point_idx, 1] + height / 2)) | np.array(pc[:, 1] < (pc[center_point_idx, 1] - height / 2)))
                                            #     condition = np.logical_not(condition)
                                            #     replace = np.random.uniform(-2, 2, 3)
                                            #     replace[2] = np.random.uniform(0, 2)
                                            #     pc[condition, :3] *= replace

                                            t = random.randint(0, 200)
                                            indexs = np.random.randint(0, pc.shape[0], t)
                                            replace = np.random.uniform(0, 1, (t, 6))
                                            replace[:, :2] = np.random.uniform(-1, 1, (t, 2))
                                            replace[:, :3] *= 2
                                            replace[:, 3:] *= 255
                                            pc[indexs, :] = replace
                                            # pc[indexs, :] *= np.random.uniform(0, 1, (t, 6))

                                            t = random.randint(0, 1000)
                                            indexs = np.random.randint(0, pc.shape[0], t)
                                            replace = np.random.uniform(0, 2, (t, 3))
                                            replace[:, 1] = replace[:, 0]
                                            replace[:, 2] = replace[:, 0]
                                            pc[indexs, 3:] *= replace
                                            pc[indexs, 3:] = np.clip(pc[indexs, 3:], 0, 255)

                                            t = random.randint(0, 200)
                                            indexs = np.random.randint(0, pc.shape[0], t)
                                            pc[indexs, 3:] = np.random.uniform(0, 1, (t, 3)) * 255
                                            # pc[indexs, 3:] *= np.random.uniform(0, 1, (t, 3))

                                        f.write(time + ".npy\n")
                                        np.save(os.path.join(self.cameraPointCloudNormDirs[i], time + ".npy"), pc)


def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--datasetDir', action='store', type=str, help='datasetDir',
                        default="/home/agilex/data", required=False)
    parser.add_argument('--episodeName', action='store', type=str, help='episodeName',
                        default="", required=False)
    parser.add_argument('--type', action='store', type=str, help='type',
                        default="aloha", required=False)
    parser.add_argument('--pointNum', action='store', type=int, help='point_num',
                        default=5000, required=False)
    parser.add_argument('--voxelSize', action='store', type=float, help='voxelSize',
                        default=0.01, required=False)
    parser.add_argument('--use_farthest_point_down_sample', action='store', type=bool, help='use_farthest_point_down_sample',
                        default=False, required=False)
    parser.add_argument('--use_augment', action='store', type=bool, help='use_augment',
                        default=False, required=False)
    parser.add_argument('--cameraNames', action='store', type=str, help='cameraNames',
                        default=[], required=False)
    args = parser.parse_args()

    with open(f'../config/{args.type}_data_params.yaml', 'r') as file:
        yaml_data = yaml.safe_load(file)
        args.cameraNames = yaml_data['/**']['ros__parameters']['dataInfo']['camera']['depth']['names']

    return args


def main():
    args = get_arguments()
    if args.episodeName == "":
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
