#! /usr/bin/env python3.10
VERSION = "0.0.1"

import argparse
from logging import shutdown
import os
import yaml
import json
import array
import numpy as np
import sys
import time

import rclpy
from rclpy.node import Node
import rclpy.time
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rclpy.serialization import serialize_message
from tf_transformations import quaternion_from_euler

from sensor_msgs.msg import Image, CompressedImage, CameraInfo, JointState, Imu
from geometry_msgs.msg import PoseStamped
from data_msgs.msg import Gripper
from nav_msgs.msg import Odometry
# from bt_task_msgs.msg import LiftMotorMsg
from tf2_msgs.msg import TFMessage


SENSOR_TOPIC_TYPE_DICT = {
    "camera_color" : CompressedImage,
    "camera_color_info" : CameraInfo,
    "camera_depth" : CompressedImage,
    "camera_depth_info" : CameraInfo,
    "arm" : JointState,
    "localization" : PoseStamped,
    "gripper" : Gripper,
    "imu" : Imu,
    "robot_base" : Odometry,
    # "lift_motor" : LiftMotorMsg,
    "tf" : TFMessage,
}
SENSOR_TOPIC_FILE_EXTENSION_DICT = {
    "camera_color" : ".jpg",
    "camera_color_info" : ".json",
    "camera_depth" : ".png",
    "camera_depth_info" : ".json",
    "arm" : ".json",
    "localization" : ".json",
    "gripper" : ".json",
    "imu" : ".json",
    "robot_base" : ".json",
    # "lift_motor" : LiftMotorMsg,
    "tf" : ".json",
}
TYPE_NAME_DICT = {
    Image : "sensor_msgs/msg/Image",
    CompressedImage : "sensor_msgs/msg/CompressedImage",
    CameraInfo : "sensor_msgs/msg/CameraInfo",
    JointState : "sensor_msgs/msg/JointState",
    PoseStamped : "geometry_msgs/msg/PoseStamped",
    Gripper : "data_msgs/msg/Gripper",
    Imu : "sensor_msgs/msg/Imu",
    Odometry : "nav_msgs/msg/Odometry",
    # LiftMotorMsg : "bt_task_msgs/msg/LiftMotorMsg",
    TFMessage : "tf2_msgs/msg/TFMessage",
}

def run_shell_command(cmd="", enable_error_output=True):
    if not enable_error_output:
        cmd += " 2> /dev/null"
    return os.popen(cmd).read()

class AlohaDataToMcapNode(Node):
    def __init__(self, args):
        super().__init__('aloha_data_to_mcap_node')
        self.datasetDir = args.datasetDir
        self.outputDir =  args.outputDir
        self.episodeIndex = args.episodeIndex
        self.alohaYaml =  args.alohaYaml
        self.in_dir = os.path.join(self.datasetDir, f"episode{self.episodeIndex}")
        self.out_dir = os.path.join(self.outputDir, f"episode{self.episodeIndex}")
        self.trans_topic_list = []
        self.batch = []
        self.sensor_info_msgs = {}
        self.camear_info_type_names = ["camera_color_info", "camera_depth_info"]

        self.save_topic_dirs, self.sensor_topic_dict, self.topic_sensor_dict, self.bind_topic_and_info_dict = self.get_topic_config_from_yaml(self.alohaYaml)
        self.init_mcap()
        self.init_file_list()

    def init_mcap(self):
        print(f"save mcap to '{self.out_dir}'")
        if os.path.exists(self.out_dir):
            print(run_shell_command(f"rm -r {self.out_dir}"))
        storage_options_output = StorageOptions(uri=self.out_dir, storage_id="mcap")
        self.mcap_writer = SequentialWriter()
        try:
            self.mcap_writer.open(storage_options_output, ConverterOptions("", ""))
        except RuntimeError:
            print(f"[error] can't wirte data to {self.mcap_path}")
            exit(-1)

        for sensor_type, topic_list in self.sensor_topic_dict.items():
            for topic in topic_list:
                topic_type_name = TYPE_NAME_DICT[SENSOR_TOPIC_TYPE_DICT[sensor_type]]
                self.create_topic_to_mcap(topic, topic_type_name)

    def init_file_list(self):
        for sensor_type in self.camear_info_type_names:
            for topic in self.sensor_topic_dict[sensor_type]:
                config_path = os.path.join(self.in_dir, self.save_topic_dirs[topic], f"config{SENSOR_TOPIC_FILE_EXTENSION_DICT[sensor_type]}")
                self.sensor_info_msgs[topic] = self.get_camera_info_msg(config_path)

        for topic, load_dir in self.save_topic_dirs.items():
            self.trans_topic_list.extend(self.get_files_by_topic(os.path.join(self.in_dir, load_dir), topic)) 
        self.trans_topic_list = sorted(self.trans_topic_list, key=lambda x: x[0])

    def add_camera_color_data(self, topic, file_path):
        frame_time = self.get_timestamp_from_file(file_path)
        with open(file_path, 'rb') as image_file:
            image_data = image_file.read()
            msg = CompressedImage()
            msg.header.stamp = frame_time.to_msg()
            msg.format = "jpeg"
            msg.data = array.array("B", image_data)
            self.push_data_to_mcap(msg, topic, frame_time.nanoseconds)
        self.add_config_data(topic, frame_time)
    
    def add_camera_depth_data(self, topic, file_path):
        frame_time = self.get_timestamp_from_file(file_path)
        with open(file_path, 'rb') as depth_file:
            depth_data = depth_file.read()
            msg = CompressedImage()
            msg.header.stamp = frame_time.to_msg()
            msg.format = "png"
            msg.data = array.array("B", depth_data)
            self.push_data_to_mcap(msg, topic, frame_time.nanoseconds)
        self.add_config_data(topic, frame_time)

    def add_config_data(self, topic, frame_time):
        info_topic = self.bind_topic_and_info_dict[topic]
        if info_topic in self.sensor_info_msgs.keys():
            info_msg = self.sensor_info_msgs[info_topic]
            info_msg.header.stamp = frame_time.to_msg()
            self.push_data_to_mcap(info_msg, info_topic, frame_time.nanoseconds)

    def add_arm_data(self, topic, file_path):
        frame_time = self.get_timestamp_from_file(file_path)
        with open(file_path, 'r') as json_file:
            data = json.load(json_file)
            msg = JointState()
            msg.header.stamp = frame_time.to_msg()
            msg.name = [f'joint{i}' for i in range(len(data["position"]))]
            msg.position = array.array("d", data["position"])
            msg.effort = array.array("d", data["effort"])
            msg.velocity = array.array("d", data["velocity"])
            self.push_data_to_mcap(msg, topic, frame_time.nanoseconds)

    def add_localization_data(self, topic, file_path):
        frame_time = self.get_timestamp_from_file(file_path)
        with open(file_path, 'r') as json_file:
            data = json.load(json_file)
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = frame_time.to_msg()
            quat = quaternion_from_euler(data["x"], data["y"], data["z"])
            msg.pose.position.x = float(data["x"])
            msg.pose.position.y = float(data["y"])
            msg.pose.position.z = float(data["z"])
            msg.pose.orientation.x = quat[0]
            msg.pose.orientation.y = quat[1]
            msg.pose.orientation.z = quat[2]
            msg.pose.orientation.w = quat[3]
            self.push_data_to_mcap(msg, topic, frame_time.nanoseconds)

    def add_gripper_data(self, topic, file_path):
        frame_time = self.get_timestamp_from_file(file_path)
        with open(file_path, 'r') as json_file:
            data = json.load(json_file)
            msg = Gripper()
            msg.header.stamp = frame_time.to_msg()
            msg.angle = float(data["angle"])
            msg.distance = float(data["distance"])
            self.push_data_to_mcap(msg, topic, frame_time.nanoseconds)

    def add_imu_data(self, topic, file_path):
        frame_time = self.get_timestamp_from_file(file_path)
        with open(file_path, 'r') as json_file:
            data = json.load(json_file)
            msg = Imu()
            msg.header.stamp = frame_time.to_msg()
            # print(type(data["angular_velocity"]))
            # print(type(msg.angular_velocity))
            # rclpy.shutdown()
            angular_velocity_dict = data["angular_velocity"]
            linear_acceleration_dict = data["linear_acceleration"]
            orientation_dict = data["orientation"]
            msg.angular_velocity.x = float(angular_velocity_dict["x"])
            msg.angular_velocity.y = float(angular_velocity_dict["y"])
            msg.angular_velocity.z = float(angular_velocity_dict["z"])
            msg.linear_acceleration.x = float(linear_acceleration_dict["x"])
            msg.linear_acceleration.y = float(linear_acceleration_dict["y"])
            msg.linear_acceleration.z = float(linear_acceleration_dict["z"])
            msg.orientation.x = float(orientation_dict["x"])
            msg.orientation.y = float(orientation_dict["y"])
            msg.orientation.z = float(orientation_dict["z"])
            msg.orientation.w = float(orientation_dict["w"])
            # msg.angular_velocity = array.array("d", data["angular_velocity"])
            # msg.linear_acceleration = array.array("d", data["linear_acceleration"])
            # msg.orientation = array.array("d", data["orientation"])
            self.push_data_to_mcap(msg, topic, frame_time.nanoseconds)

    def add_robot_base_data(self, topic, file_path):
        frame_time = self.get_timestamp_from_file(file_path)
        pass

    def add_lift_motor_data(self, topic, file_path):
        frame_time = self.get_timestamp_from_file(file_path)
        pass

    def add_tf_data(self, topic, file_path):
        frame_time = self.get_timestamp_from_file(file_path)
        pass

    def get_timestamp_from_file(self, file_path):
        base_name, extension = os.path.splitext(os.path.basename(file_path))
        return rclpy.time.Time(seconds=float(base_name))

    def create_topic_to_mcap(self, topic_name, topic_type):
        topic_metadata = TopicMetadata(topic_name, topic_type, 'cdr')
        self.mcap_writer.create_topic(topic_metadata)

    def write_data_to_mcap(self):
        for b_topic, b_data, b_timestamp in self.batch:
            self.mcap_writer.write(b_topic, b_data, b_timestamp)
        self.batch = []

    def push_data_to_mcap(self, data, topic_name, timestamp):
        self.batch.append((topic_name, serialize_message(data), timestamp))
        if len(self.batch) >= 2000:
            self.write_data_to_mcap()

    def get_topic_config_from_yaml(self, yaml_path):
        if not os.path.exists(yaml_path):
            return {}, {}
        topic_dir = {}
        sensor_topic_dict = {}
        topic_sensor_dict = {}
        bind_topic_and_info_dict = {}
        with open(yaml_path, "r") as f:
            yaml_dict = yaml.safe_load(f)
            data_info_dict = self.get_dict_from_index(yaml_dict, "dataInfo")
            for key1, value1 in data_info_dict.items():
                for key2, value2 in value1.items():
                    names_list = value2["names"]
                    topics_list = value2["topics"]
                    sensor_type = key1
                    if len(data_info_dict[key1].keys()) > 1:
                        sensor_type = f"{key1}_{key2}"
                    sensor_topic_dict[sensor_type] = topics_list
                    for i in range(len(topics_list)):
                        topic_dir[topics_list[i]] = f"{key1}/{key2}/{names_list[i]}"
                        topic_sensor_dict[topics_list[i]] = sensor_type
                    if "configTopics" in value2.keys():
                        config_topics_list = value2["configTopics"]
                        sensor_topic_dict[f"{sensor_type}_info"] = config_topics_list
                        for i in range(len(config_topics_list)):
                            topic_dir[config_topics_list[i]] = f"{key1}/{key2}/{names_list[i]}"
                            bind_topic_and_info_dict[topics_list[i]] = config_topics_list[i]
        return topic_dir, sensor_topic_dict, topic_sensor_dict, bind_topic_and_info_dict

    def get_dict_from_index(self, data, index=""):
        for key, value in data.items():
            if isinstance(value, dict):
                if key == index:
                    return value
                else:
                    res = self.get_dict_from_index(value, index)
                    if len(res) != 0:
                        return res
        return dict()

    def get_camera_info_msg(self, info_path) -> CameraInfo:
        info_msg = CameraInfo()
        with open(info_path, 'r') as info_file:
            data = json.load(info_file)
            info_msg.d = array.array('d', data["D"])
            info_msg.k = np.array(data["K"], dtype=np.float64)
            info_msg.p = np.array(data["P"], dtype=np.float64)
            info_msg.r = np.array(data["R"], dtype=np.float64)
            info_msg.binning_x = data["binning_x"]
            info_msg.binning_y = data["binning_y"]
            info_msg.distortion_model = data["distortion_model"]
            info_msg.height = data["height"]
            info_msg.width = data["width"]
        return info_msg

    def get_files_by_topic(self, directory, topic="/camera_l/color/image_raw"):
        if topic not in self.topic_sensor_dict.keys():
            return []

        extension = SENSOR_TOPIC_FILE_EXTENSION_DICT[self.topic_sensor_dict[topic]]
        files = [f for f in os.listdir(directory) if f.endswith(extension)]
        files_with_labels =  [[f, topic] for f in files]
        return files_with_labels

    def process(self):
        start_time = time.time()
        total_count = len(self.trans_topic_list)
        count = 0
        for file, topic in self.trans_topic_list:
            count += 1
            sys.stdout.write('\r[%3d%%] frame progress ( %d / %d )' % (int(count/total_count*100), count, total_count))
            sys.stdout.flush()
            file_path = os.path.join(self.in_dir, self.save_topic_dirs[topic], file)
            sensor_type = self.topic_sensor_dict[topic]
            getattr(self, f"add_{sensor_type}_data")(topic, file_path)
        self.write_data_to_mcap()
        print()
        end_time = time.time()
        print("spend time: %-.3fs" % (end_time - start_time))

def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--datasetDir', action='store', type=str, help='datasetDir.',
                        default="", required=False)
    parser.add_argument('--outputDir', action='store', type=str, help='outputDir.',
                        default="", required=False)
    parser.add_argument('--episodeIndex', action='store', type=int, help='Episode index.',
                        default=0, required=False)
    parser.add_argument('--alohaYaml', action='store', type=str, help='alohaYaml.',
                        default="/home/agilex/ros2_ws/src/data_tools/config/aloha_data_params.yaml", required=False)
    args = parser.parse_args()

    if args.outputDir == "" or args.outputDir == args.datasetDir:
        args.outputDir = os.path.join(args.datasetDir, "mcap")
    print("args:")
    print(f" --datasetDir: {args.datasetDir}")
    print(f" --outputDir: {args.outputDir}")
    print(f" --episodeIndex: {args.episodeIndex}")
    print(f" --alohaYaml: {args.alohaYaml}")
    return args

def main():
    global HOME
    if "HOME" in os.environ:
        HOME = os.environ["HOME"]
    else:
        HOME = "/home/agilex"

    args = get_arguments()
    rclpy.init()
    node = AlohaDataToMcapNode(args)
    node.process()
    rclpy.shutdown()

    # try:
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     node.destroy_node()
    #     if rclpy.ok():
    #         rclpy.shutdown()

if __name__ == "__main__":
    main()