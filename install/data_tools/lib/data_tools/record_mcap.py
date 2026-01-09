#! /usr/bin/env python3.10
VERSION = "0.0.1"
import os
import threading
import yaml
import functools
import time
import copy
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image, CompressedImage, CameraInfo, JointState, Imu
from geometry_msgs.msg import PoseStamped
from data_msgs.msg import Gripper
from nav_msgs.msg import Odometry
# from bt_task_msgs.msg import LiftMotorMsg
from tf2_msgs.msg import TFMessage
from data_msgs.msg import CaptureStatus
from data_msgs.srv import CaptureService

SUB_COMPRESSED_CAMERA = True

SENSOR_TOPIC_TYPE_DICT = {
    "camera_color" : CompressedImage,
    "camera_color_info" : CameraInfo,
    "camera_depth" : CompressedImage,
    "camera_depth_info" : CameraInfo,
    "arm" : JointState,
    "localization" : PoseStamped,
    "gripper" : Gripper,
    "imu" : Imu,
    "robotBase" : Odometry,
    # "lift" : LiftMotorMsg,
    "transform_tf" : TFMessage,
    "transform_tf_static" : TFMessage,
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

class CountInfo:
    def __init__(self):
        self.init_time = False
        self.count = 0
        self.start_time = 0
        self.last_time = 0

    def add_count_and_time(self, timestamp):
        self.count += 1
        self.set_timestamp(timestamp)

    def set_timestamp(self, timestamp):
        if not self.init_time:
            self.init_time = True
            self.start_time = timestamp
        else:
            self.last_time = timestamp

    def get_count(self):
        return self.count

    def get_frequency(self):
        if self.init_time and self.last_time - self.start_time != 0:
            return self.count / (self.last_time - self.start_time)
        return 0

    def count_frequency(self, start_time, end_time):
        if self.init_time and end_time - start_time != 0:
            return self.count / (end_time - start_time)
        return 0

class Rate:
    def __init__(self, rate):
        self.sleep_time = 1.0 / rate
        self.start_time = time.time()

    def sleep(self):
        if (self.start_time+self.sleep_time > time.time()):
            time.sleep(self.start_time+self.sleep_time-time.time())
            self.start_time += self.sleep_time
        else:
            self.start_time = time.time()

class RecordMcapNode(Node):
    def __init__(self):
        super().__init__("record_mcap")
        self.thread_run = False
        self.declare_parameter('episodeIndex', 0)
        self.declare_parameter('datasetDir', "/home/agilex/data-ario")
        self.declare_parameter('paramsFile', "/home/agilex/ros2_ws/src/data_tools/config/aloha_data_params.yaml")
        self.declare_parameter('hz', 20)
        self.declare_parameter('timeout', 2)
        self.declare_parameter('useService', False)
        self.declare_parameter('useTopicStamp', False)
        self.episode_index = self.get_parameter('episodeIndex').get_parameter_value().integer_value
        self.dataset_dir = self.get_parameter('datasetDir').get_parameter_value().string_value
        self.topic_config = self.get_parameter('paramsFile').get_parameter_value().string_value
        self.hz = self.get_parameter('hz').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().integer_value
        self.use_service = self.get_parameter('useService').get_parameter_value().bool_value
        self.use_topic_stamp = self.get_parameter('useTopicStamp').get_parameter_value().bool_value
        self.get_logger().info(f"Version: {VERSION}")
        self.get_logger().info(f"episodeIndex: {self.episode_index}")
        self.get_logger().info(f"datasetDir: {self.dataset_dir}")
        self.get_logger().info(f"paramsFile: {self.topic_config}")
        self.get_logger().info(f"hz: {self.hz}")
        self.get_logger().info(f"timeout: {self.timeout}")
        self.get_logger().info(f"useService: {self.use_service}")
        self.get_logger().info(f"useTopicStamp: {self.use_topic_stamp}")
        if self.use_service:
            print("param 'useService' is true, wait for start signal.\n")
            self.capture_srv = self.create_service(CaptureService, '/data_tools_dataCapture/capture_service', self.handle_caputure_service) 
        else:
            self.start_record_mcap()

    def handle_caputure_service(self, req, res):
        res.success = True
        res.message = ""
        if req.start:
            if self.thread_run:
                episode_dir = os.path.join(self.dataset_dir, f"episode{self.episode_index}")
                res.success = False
                res.message = f"capture progress is running: {episode_dir}"
            else:
                if req.dataset_dir != "":
                    self.dataset_dir = req.dataset_dir
                if req.episode_index != -1:
                    self.episode_index = req.episode_index
                print(f"start capture progress.")
                print("====================")
                self.get_logger().info(f"episodeIndex: {self.episode_index}")
                self.get_logger().info(f"datasetDir: {self.dataset_dir}")
                self.start_record_mcap()
                self.episode_index += req.episode_index == -1
                res.message = f"start capture progress success."
        elif req.end:
            if self.thread_run:
                self.stop_record_mcap()
                print("--------------------")
                print(f"\nwait for start signal.\n")
                res.message = f"close capture progress success."
            else:
                res.success = False
                res.message = f"capture progress isn't running"
        return res

    def topic_callback(self, msg, topic_name):
        cur_stamp = self.get_clock().now().to_msg()
        if self.use_topic_stamp:
            if topic_name == "/tf":
                stamp = msg.transforms[0].header.stamp
            elif topic_name == "/tf_static":
                stamp = cur_stamp
            else:
                stamp = msg.header.stamp
        else:
            stamp = cur_stamp
        timestamp = stamp.sec + stamp.nanosec / 1e9
        self.topic_count_dict[topic_name].add_count_and_time(timestamp)
        self.mcap_writer.write(topic_name, serialize_message(msg), int(timestamp * 1e9))
        if self.topic_start_time == 0:
            self.topic_start_time = timestamp
        self.topic_last_time = timestamp

    def start_record_mcap(self):
        self.print_line = 0
        self.topic_start_time = self.topic_last_time = 0
        self.init_topic_info_and_mcap()
        self.start_sub_and_thread()

    def init_topic_info_and_mcap(self):
        self.sensor_topic_dict, self.topic_count_dict, self.topic_hz_dict = self.get_topic_config_from_yaml(self.topic_config)
        # print(self.sensor_topic_dict)
        self.last_topic_count_dict = copy.deepcopy(self.topic_count_dict)
        self.mcap_path = os.path.join(self.dataset_dir, f"episode{self.episode_index}")
        print(f"save mcap to '{self.mcap_path}'")
        if os.path.exists(self.mcap_path):
            print(run_shell_command(f"rm -r {self.mcap_path}"))
        storage_options_output = StorageOptions(uri=self.mcap_path, storage_id="mcap")
        self.mcap_writer = SequentialWriter()
        try:
            self.mcap_writer.open(storage_options_output, ConverterOptions("", ""))
        except RuntimeError:
            print(f"[error] can't wirte data to {self.mcap_path}")
            return

    def start_sub_and_thread(self):
        self.capture_status_pub = self.create_publisher(CaptureStatus, "/data_tools_dataCapture/status", 2000)
        self.topic_sub_dict = {}
        for sensor_type, topic_list in self.sensor_topic_dict.items():
            for i in range(len(topic_list)):
                mcap_topic_name = sub_topic_name = topic_list[i]
                mcap_topic_type = sub_topic_type = SENSOR_TOPIC_TYPE_DICT[sensor_type]
                if SUB_COMPRESSED_CAMERA and (sensor_type == "camera_color" or sensor_type ==  "camera_depth"):
                    sub_topic_name += "/compressed"
                self.create_topic_to_mcap(mcap_topic_name, TYPE_NAME_DICT[mcap_topic_type])
                qos_profile = QoSProfile(depth=10)
                if mcap_topic_name == "/tf_static":
                    # 锁存话题，后续新增一个锁存列表来判断
                    qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
                self.topic_sub_dict[mcap_topic_name] = self.create_subscription(sub_topic_type, sub_topic_name, functools.partial(self.topic_callback, topic_name=mcap_topic_name), qos_profile)
        self.thread_run = True
        self.thread = threading.Thread(target=self.process)
        self.thread.start()

    def stop_record_mcap(self):
        self.thread_run = False
        self.destroy_publisher(self.capture_status_pub)
        for i in self.topic_sub_dict:
            self.destroy_subscription(self.topic_sub_dict[i])
        self.mcap_writer.close()
        path = os.path.join(self.mcap_path, f"episode{self.episode_index}_0.mcap")
        print(f"---\n{path} info:\n")
        print(run_shell_command(f"mcap info {path}"))

    def create_topic_to_mcap(self, topic_name, topic_type):
        if topic_name != "/tf_static":
            topic_metadata = TopicMetadata(topic_name, topic_type, 'cdr')
        else:
            topic_metadata = TopicMetadata(topic_name, topic_type, 'cdr', "- history: 1\n  depth: 10\n  reliability: 1\n  durability: 1\n  deadline:\n    sec: 0\n    nsec: 0\n  lifespan:\n    sec: 0\n    nsec: 0\n  liveliness: 1\n  liveliness_lease_duration:\n    sec: 0\n    nsec: 0\n  avoid_ros_namespace_conventions: false")
        self.mcap_writer.create_topic(topic_metadata)

    def get_topic_config_from_yaml(self, yaml_path):
        if not os.path.exists(yaml_path):
            return {}, {}
        sensor_topic_dict = {}
        topic_count_dict = {}
        topic_hz_dict = {}
        with open(yaml_path, "r") as f:
            yaml_dict = yaml.safe_load(f)
            data_info_dict = self.get_dict_from_index(yaml_dict, "dataInfo")
            for key1, value1 in data_info_dict.items():
                for key2, value2 in value1.items():
                    check_hz = self.hz
                    sensor_type = key1
                    topics_list = value2["topics"]
                    if "checkFrameRate" in value2.keys():
                        check_hz = value2["checkFrameRate"]
                    sensor_type = key1
                    if len(data_info_dict[key1].keys()) > 1:
                        sensor_type = f"{key1}_{key2}"
                    sensor_topic_dict[sensor_type] = topics_list
                    for i in range(len(topics_list)):
                        topic_count_dict[topics_list[i]] = CountInfo()
                        topic_hz_dict[topics_list[i]] = check_hz
                    if "configTopics" in value2.keys():
                        config_topics_list = value2["configTopics"]
                        sensor_topic_dict[f"{sensor_type}_info"] = config_topics_list
                        for i in range(len(config_topics_list)):
                            topic_count_dict[config_topics_list[i]] = CountInfo()
                            topic_hz_dict[config_topics_list[i]] = check_hz
        return sensor_topic_dict, topic_count_dict, topic_hz_dict

    def get_dict_from_index(self, data, index=""):
        for key, value in data.items():
            if isinstance(value, dict):
                if key == index:
                    return value
                else:
                    res = self.get_dict_from_index(value, index)
                    if len(res) != 0:
                        return res
        return {}

    def get_cur_ros_time_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def print_and_count_line(self, message):
        self.print_line += 1
        print(message)

    def clear_print(self):
        sys.stdout.write("\033[F\033[K" * self.print_line)
        sys.stdout.flush()
        self.print_line = 0

    def process(self):
        self.rate = Rate(1)
        topic_last_up_time_dict = {}
        start_time_sec = self.get_cur_ros_time_sec()
        while (rclpy.ok() and self.thread_run):
            cur_time_sec = self.get_cur_ros_time_sec()
            self.print_and_count_line(f"--- {int(cur_time_sec-start_time_sec)} ---")
            msg = CaptureStatus()
            for topic_name in self.topic_count_dict.keys():
                cur_count = self.topic_count_dict[topic_name].get_count()
                last_count = self.last_topic_count_dict[topic_name].get_count()
                count_in_sec = cur_count - last_count
                cur_frequency = int(round(self.topic_count_dict[topic_name].count_frequency(self.topic_start_time, self.topic_last_time)*100)) / 100
                sys.stdout.write(f"{topic_name}: {count_in_sec} / {cur_count} ({cur_frequency}HZ)")
                if topic_name not in topic_last_up_time_dict.keys():
                    topic_last_up_time_dict[topic_name] = cur_time_sec
                check_hz = self.topic_hz_dict[topic_name]
                if count_in_sec < check_hz or cur_frequency < check_hz:
                    if (cur_time_sec - topic_last_up_time_dict[topic_name] > self.timeout):
                        self.print_and_count_line(f" < ({check_hz}HZ) --- [check]")
                        msg.fail = True
                    else:
                        self.print_and_count_line("")
                else:
                    self.print_and_count_line("")
                    topic_last_up_time_dict[topic_name] = cur_time_sec
                msg.topics.append(topic_name)
                msg.frequencies.append(cur_frequency)
                msg.count_in_seconds.append(count_in_sec)
            self.capture_status_pub.publish(msg)

            self.last_topic_count_dict = copy.deepcopy(self.topic_count_dict)
            self.rate.sleep()
            if rclpy.ok() and self.thread_run:
                self.clear_print()
        self.thread_run = False

def main(args=None):
    rclpy.init(args=args)
    node = RecordMcapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()