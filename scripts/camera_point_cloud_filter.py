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

class Operator:
    def __init__(self, args):
        self.args = args
        self.episodeDir = os.path.join(self.args.datasetDir, "episode" + str(self.args.episodeIndex))
        self.cameraPointCloudDirs = [os.path.join(self.episodeDir, "camera/pointCloud/" + self.args.cameraPointCloudNames[i]) for i in range(len(self.args.cameraPointCloudNames))]
        self.cameraPointCloudNormDirs = [os.path.join(self.episodeDir, "camera/pointCloud/" + self.args.cameraPointCloudNames[i] + "-normalization") for i in range(len(self.args.cameraPointCloudNames))]
        self.cameraPointCloudSyncDirs = [os.path.join(self.cameraPointCloudDirs[i], "sync.txt") for i in range(len(self.args.cameraPointCloudNames))]
        self.cameraPointCloudNormSyncDirs = [os.path.join(self.cameraPointCloudNormDirs[i], "sync.txt") for i in range(len(self.args.cameraPointCloudNames))]

    def farthest_point_sampling(self, points, k):
        sampled_points = [np.random.randint(len(points))]
        distances = np.linalg.norm(points - points[sampled_points[-1]], axis=1)
        for _ in range(k - 1):
            farthest_index = np.argmax(distances)
            sampled_points.append(farthest_index)
            distances = np.minimum(distances, np.linalg.norm(points - points[farthest_index], axis=1))
        return sampled_points

    def process(self):
        for i in range(len(self.args.cameraPointCloudNames)):
            os.system(f"rm -rf {self.cameraPointCloudNormDirs[i]}")
            os.system(f"mkdir {self.cameraPointCloudNormDirs[i]}")
            os.system(f"cp {self.cameraPointCloudDirs[i]}/config.json {self.cameraPointCloudNormDirs[i]}/config.json")
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
                            if self.args.use_farthest_point_down_sample and len(downsampled_cloud.points) > self.args.pointNum:
                                downsampled_cloud = downsampled_cloud.farthest_point_down_sample(self.args.pointNum)

                            downsampled_cloud.colors = o3d.utility.Vector3dVector((np.asarray(downsampled_cloud.colors) * 255).astype(np.float64))
                            pc = np.concatenate([downsampled_cloud.points, downsampled_cloud.colors], axis=-1)
                            condition = pc[:, 2] < 1
                            pc = pc[condition, :]

                            if pc.shape[0] > self.args.pointNum:
                                idxs = np.random.choice(pc.shape[0], self.args.pointNum, replace=False)
                                pc = pc[idxs]
                            elif pc.shape[0] < self.args.pointNum:
                                if pc.shape[0] == 0:
                                    pc = np.zeros([1, 4], dtype=np.float32)
                                idxs1 = np.arange(pc.shape[0])
                                idxs2 = np.random.choice(pc.shape[0], self.args.pointNum - pc.shape[0], replace=True)
                                idxs = np.concatenate([idxs1, idxs2], axis=0)
                                pc = pc[idxs]
                        else:
                            pc = pcl.load_XYZRGB(os.path.join(self.cameraPointCloudDirs[i], line)).to_array()
                            condition = pc[:, 2] < 1
                            pc = pc[condition, :]
                            if pc.shape[0] >= self.args.pointNum:
                                idxs = np.random.choice(pc.shape[0], self.args.pointNum, replace=False)
                            elif pc.shape[0] < self.args.pointNum:
                                if pc.shape[0] == 0:
                                    pc = np.zeros([1, 4], dtype=np.float32)
                                idxs1 = np.arange(pc.shape[0])
                                idxs2 = np.random.choice(pc.shape[0], self.args.pointNum - pc.shape[0], replace=True)
                                idxs = np.concatenate([idxs1, idxs2], axis=0)

                            rgbs = pc[idxs][:, 3].view(np.uint32)
                            r = (np.right_shift(rgbs, 16) % 256)[:, np.newaxis]
                            g = (np.right_shift(rgbs, 8) % 256)[:, np.newaxis]
                            b = (rgbs % 256)[:, np.newaxis]
                            r_g_b = np.concatenate([r, g, b], axis=-1)
                            # rgbs = pc[idxs][:, 3]
                            # r_g_b = []
                            # for index in range(rgbs.shape[0]):
                            #     rgb_float32 = rgbs[index]
                            #     packed = struct.unpack('I', struct.pack('f', rgb_float32))[0]
                            #     r = (packed >> 16) & 0x0000ff
                            #     g = (packed >> 8) & 0x0000ff
                            #     b = (packed) & 0x0000ff
                            #     r_g_b.append([r, g, b])
                            # r_g_b = np.array(r_g_b, dtype=np.float32)
                            pc = np.concatenate([pc[idxs][:, :3], r_g_b], axis=-1)

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

                        f.write(time+".npy\n")
                        np.save(os.path.join(self.cameraPointCloudNormDirs[i], time+".npy"), pc)


def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--datasetDir', action='store', type=str, help='datasetDir',
                        default="/home/agilex/data", required=False)
    parser.add_argument('--episodeIndex', action='store', type=int, help='episodeIndex',
                        default=-1, required=False)
    parser.add_argument('--pointNum', action='store', type=int, help='point_num',
                        default=5000, required=False)
    parser.add_argument('--voxelSize', action='store', type=float, help='voxelSize',
                        default=0.01, required=False)
    parser.add_argument('--use_farthest_point_down_sample', action='store', type=bool, help='use_farthest_point_down_sample',
                        default=False, required=False)
    parser.add_argument('--use_augment', action='store', type=bool, help='use_augment',
                        default=False, required=False)
    parser.add_argument('--cameraPointCloudNames', action='store', type=str, help='cameraPointCloudNames',
                        default=['pikaDepthCamera'], required=False)
    args = parser.parse_args()
    return args


def main():
    args = get_arguments()
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
