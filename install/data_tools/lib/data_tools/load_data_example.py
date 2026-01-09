import numpy as np
import torch
import os
import h5py
from torch.utils.data import TensorDataset, DataLoader
import argparse


def flatten_list(target):
    return [item for sublist in target for item in sublist]


def find_all_hdf5(dataset_dir):
    hdf5_files = []
    for f in os.listdir(dataset_dir):
        hdf5_files.append(os.path.join(os.path.join(dataset_dir, f), "data.hdf5"))
    print(f'Found {len(hdf5_files)} hdf5 files')
    return hdf5_files


def batch_sampler(batch_size, episode_len_list):
    sum_dataset_len_l = np.cumsum([0] + [np.sum(episode_len) for episode_len in episode_len_list])
    while True:
        batch = []
        for _ in range(batch_size):
            episode_idx = np.random.choice(len(episode_len_list))
            step_idx = np.random.randint(sum_dataset_len_l[episode_idx], sum_dataset_len_l[episode_idx + 1])
            batch.append(step_idx)
        yield batch


def get_all_episode_len(dataset_path_list):
    all_episode_len = []
    for dataset_path in dataset_path_list:
        try:
            with h5py.File(dataset_path, 'r') as root:
                all_episode_len.append(root['size'][()])
        except Exception as e:
            print(e)
            quit()
    return all_episode_len


class EpisodicDataset(torch.utils.data.Dataset):

    def __init__(self, dataset_path_list, episode_ids, episode_len):
        super(EpisodicDataset).__init__()
        self.dataset_path_list = dataset_path_list
        self.episode_ids = episode_ids
        self.episode_len = episode_len
        self.cumulative_len = np.cumsum(self.episode_len)
        self.max_episode_len = max(episode_len)
        self.transformations = None
        self.__getitem__(0)

    def _locate_transition(self, index):
        assert index < self.cumulative_len[-1]
        episode_index = np.argmax(self.cumulative_len > index)  # argmax returns first True index
        start_index = index - (self.cumulative_len[episode_index] - self.episode_len[episode_index])
        episode_id = self.episode_ids[episode_index]
        return episode_id, start_index

    def __getitem__(self, index):
        episode_id, start_index = self._locate_transition(index)
        dataset_path = self.dataset_path_list[episode_id]
        # print(episode_id, start_index)
        with h5py.File(dataset_path, 'r') as root:
            # qpos = np.concatenate((root['/arm/jointStatePosition/puppetLeft'][()], root['/arm/jointStatePosition/puppetRight'][()]), axis=1)
            # action = np.concatenate((root['/arm/jointStatePosition/masterLeft'][()], root['/arm/jointStatePosition/masterRight'][()]), axis=1)
            if '/localization/pose/pika' in root.keys():
                qpos = root['/localization/pose/pika'][()][start_index]
                action = root['/localization/pose/pika'][()][start_index]
            elif '/localization/pose/pika_l' in root.keys():
                qpos = np.concatenate((root['/localization/pose/pika_l'][()], root['/localization/pose/pika_r'][()]), axis=-1)[start_index]
                action = np.concatenate((root['/localization/pose/pika_l'][()], root['/localization/pose/pika_r'][()]), axis=-1)[start_index]
            elif '/arm/jointStatePosition/masterLeft' in root.keys():
                action = np.concatenate((root['/arm/jointStatePosition/masterLeft'][()], root['/arm/jointStatePosition/masterRight'][()]), axis=-1)[start_index]
                qpos = np.concatenate((root['/arm/jointStatePosition/puppetLeft'][()], root['/arm/jointStatePosition/puppetRight'][()]), axis=-1)[start_index]
            # qpos = torch.from_numpy(qpos[start_index]).float()
            # action = torch.from_numpy(action[start_index]).float()
            # root['/arm/endPose/puppetLeft'][()]  根据需要获取
            # root['/arm/endPose/puppetRight'][()]
            # root['/arm/endPose/masterLeft'][()]
            # root['/arm/endPose/masterRight'][()]
            # root['/arm/jointStatePosition/puppetLeft'][()]
            # root['/arm/jointStatePosition/puppetRight'][()]
            # root['/arm/jointStatePosition/masterLeft'][()]
            # root['/arm/jointStatePosition/masterRight'][()]
            # cv2.imread(os.path.join(dataset_path[:-9], root[f'/camera/color/left'][start_index].decode('utf-8')), cv2.IMREAD_UNCHANGED)
            # cv2.imread(os.path.join(dataset_path[:-9], root[f'/camera/depth/left'][start_index].decode('utf-8')), cv2.IMREAD_UNCHANGED)
            # pcl.load(os.path.join(dataset_path[:-9], root[f'/camera/pointCloud/left'][start_index].decode('utf-8'))).to_array()

        return qpos, action


def load_data(dataset_dir, batch_size):
    dataset_dir_list = dataset_dir
    if type(dataset_dir_list) == str:
        dataset_dir_list = [dataset_dir_list]
    dataset_path_list_list = [find_all_hdf5(dataset_dir) for dataset_dir in dataset_dir_list]

    dataset_path_list = flatten_list(dataset_path_list_list)
    num_episodes_list = [0] + [len(dataset_path_list) for dataset_path_list in dataset_path_list_list]
    num_episodes_cumsum = np.cumsum(num_episodes_list)

    episode_ids_list = []
    # obtain train test split on dataset_dir_l[0]
    for i in range(len(dataset_path_list_list)):
        num_episodes = len(dataset_path_list_list[i])
        shuffled_episode_ids = np.random.permutation(num_episodes)
        episode_ids_list.append(np.array([train_id+num_episodes_cumsum[i] for train_id in shuffled_episode_ids]))

    all_episode_len = get_all_episode_len(dataset_path_list)
    episode_len_list = [[all_episode_len[i] for i in episode_ids] for episode_ids in episode_ids_list]

    episode_len = flatten_list(episode_len_list)
    episode_ids = np.concatenate(episode_ids_list)
    dataset = EpisodicDataset(dataset_path_list, episode_ids, episode_len)
    num_workers = 1
    dataloader = DataLoader(dataset, batch_sampler=batch_sampler(batch_size, episode_len_list),
                            pin_memory=True, num_workers=num_workers, prefetch_factor=1)

    return dataloader


def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--datasetDir', action='store', type=str, help='datasetDir',
                        default="/home/agilex/data", required=False)
    parser.add_argument('--batchSize', action='store', type=int, help='batchSize',
                        default=16, required=False)
    args = parser.parse_args()
    return args


def main():
    args = get_arguments()
    dataloader = load_data(args.datasetDir, args.batchSize)
    step = 1000
    for batch_idx, data in enumerate(dataloader):
        print(batch_idx, data[0].shape, data[1].shape)
        if batch_idx >= step:
            break

if __name__ == '__main__':
    main()
