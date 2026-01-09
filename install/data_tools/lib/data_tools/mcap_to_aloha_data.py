#! /usr/bin/env python3.10
import argparse
import json
import os

import yaml

from rosbag2_py import StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
import time
import sys
import tf_transformations

def ros_time_to_sec_str(timestamp) -> str:
    return f"{timestamp.sec+timestamp.nanosec/1e9:.6f}"
    # return f"{timestamp.sec}.{int(round(timestamp.nanosec/1000))}"

def run_shell_command(cmd="", enable_error_output=True):
    if not enable_error_output:
        cmd += " 2> /dev/null"
    return os.popen(cmd).read()

def get_mcap_msg_num(mcap_file):
    return int(run_shell_command("mcap info " + mcap_file + " | grep -w ^messages").split()[1])

def create_directory(dir_path):
    if not os.path.isdir(dir_path):
        os.makedirs(dir_path, exist_ok=True)

def print_dict(data, ss=""):
    for key, value in data.items():
        if isinstance(value, dict):
            print(f"{ss}{key}:")
            print_dict(value, f"{ss}  ")
        else:
            print(f"{ss}{key}: {value}")

def get_dict_from_index(data, index=""):
    for key, value in data.items():
        if isinstance(value, dict):
            if key == index:
                return value
            else:
                res = get_dict_from_index(value, index)
                if len(res) != 0:
                    return res
    return dict()

def get_topic_config_from_yaml(yaml_path):
    if not os.path.exists(yaml_path):
        return {}, {}

    topic_dir = {}
    sensor_type = {}
    with open(yaml_path, "r") as f:
        yaml_dict = yaml.safe_load(f)
        data_info_dict = get_dict_from_index(yaml_dict, "dataInfo")
        for key1, value1 in data_info_dict.items():
            for key2, value2 in value1.items():
                names_list = value2["names"]
                topics_list = value2["topics"]
                sensor_type_name = key1
                if len(data_info_dict[key1].keys()) > 1:
                        sensor_type_name = f"{key1}_{key2}"
                sensor_type[sensor_type_name] = topics_list
                for i in range(len(topics_list)):
                    topic_dir[topics_list[i]] = f"{key1}/{key2}/{names_list[i]}"
                if "configTopics" in value2.keys():
                    config_topics_list = value2["configTopics"]
                    sensor_type[f"{sensor_type_name}_info"] = config_topics_list
                    for i in range(len(config_topics_list)):
                        topic_dir[config_topics_list[i]] = f"{key1}/{key2}/{names_list[i]}"
    return topic_dir, sensor_type

def process_file(input_mcap_file, data_dir, yaml_path):
    msg_num = get_mcap_msg_num(input_mcap_file)

    parent_path = os.path.dirname(input_mcap_file)
    parent_dir = os.path.basename(parent_path)
    output_dir_head = f"{data_dir}/aloha/{parent_dir}/"
    print(f"\n>>> {input_mcap_file}")
    # print(f"输入mcap为 {input_mcap_file}")
    print(f"输出: {output_dir_head}")

    # 打开输入的 MCAP 文件
    storage_options_input = StorageOptions(uri=input_mcap_file, storage_id="mcap")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options_input, ConverterOptions("", ""))
    topic_types_dict = {t.name: t.type for t in reader.get_all_topics_and_types()}

    save_topic_dirs, sensor_topic_dict = get_topic_config_from_yaml(yaml_path)
    print(f"---")
    for sensor_type, topic_list in sensor_topic_dict.items():
        print(f"{sensor_type}:")
        for topic in topic_list:
            topic_dir = save_topic_dirs[topic]
            create_directory(f"{output_dir_head}{topic_dir}")
            if topic in topic_types_dict:
                print(f"  * {topic}\t[{topic_dir}]\t: {topic_types_dict[topic]}")
            else:
                print(f"  * {topic}\t[{topic_dir}]\t: (not found)")
    print("---\n")

    # 生成aloha数据
    total_count = 0
    fail_topics = []
    start_time = time.time()
    while reader.has_next():
        total_count += 1
        sys.stdout.write('\r[%3d%%] progress ( %d / %d )' % (int(total_count/msg_num*100), total_count, msg_num))
        sys.stdout.flush()

        topic, data, timestamp = reader.read_next()
        # camera color
        if "camera_color" in sensor_topic_dict and topic in sensor_topic_dict["camera_color"]:
            msg = deserialize_message(data, get_message(topic_types_dict[topic]))
            filename = ros_time_to_sec_str(msg.header.stamp)
            with open(f'{output_dir_head}{save_topic_dirs[topic]}/{filename}.jpg', 'wb') as f:
                f.write(msg.data)
        # camera depth
        elif "camera_depth" in sensor_topic_dict and topic in sensor_topic_dict["camera_depth"]:
            msg = deserialize_message(data, get_message(topic_types_dict[topic]))
            filename = ros_time_to_sec_str(msg.header.stamp)
            with open(f'{output_dir_head}{save_topic_dirs[topic]}/{filename}.png', 'wb') as f:
                f.write(msg.data)
        # camera color and depth info
        elif ("camera_color_info" in sensor_topic_dict and topic in sensor_topic_dict["camera_color_info"]) or \
            ("camera_depth_info" in sensor_topic_dict and topic in sensor_topic_dict["camera_depth_info"]):
            msg = deserialize_message(data, get_message(topic_types_dict[topic]))
            roi_dict = {
                "do_rectify" : int(msg.roi.do_rectify),
                "height" : msg.roi.height,
                "width" : msg.roi.width,
                "x_offset" : msg.roi.x_offset,
                "y_offset" : msg.roi.y_offset
            }
            camera_info_data_dict = {
                "D": msg.d.tolist(),
                "K": msg.k.tolist(),
                "P": msg.p.tolist(),
                "R": msg.r.tolist(),
                "binning_x": msg.binning_x,
                "binning_y": msg.binning_y,
                "distortion_model": msg.distortion_model,
                "height": msg.height,
                "roi": roi_dict,
                "width": msg.width
            }
            with open(f"{output_dir_head}{save_topic_dirs[topic]}/config.json", 'w+') as json_file:
                json.dump(camera_info_data_dict, json_file, indent="\t")
        # arm
        elif "arm" in sensor_topic_dict and topic in sensor_topic_dict["arm"]:
            msg = deserialize_message(data, get_message(topic_types_dict[topic]))
            filename = ros_time_to_sec_str(msg.header.stamp)
            joint_state_dict = {
                "effort" : msg.effort.tolist(),
                "position" : msg.position.tolist(),
                "velocity" : msg.velocity.tolist(),
            }
            with open(f"{output_dir_head}{save_topic_dirs[topic]}/{filename}.json", 'w+') as json_file:
                json.dump(joint_state_dict, json_file, indent="\t")
        # localization
        elif "localization" in sensor_topic_dict and topic in sensor_topic_dict["localization"]:
            msg = deserialize_message(data, get_message(topic_types_dict[topic]))
            filename = ros_time_to_sec_str(msg.header.stamp)
            position = msg.pose.position
            orientation = msg.pose.orientation
            orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
            rpy_list = tf_transformations.euler_from_quaternion(orientation_list)
            pose_dict = {
                "pitch" : rpy_list[1],
                "roll" : rpy_list[0],
                "x" : position.x,
                "y" : position.y,
                "yaw" : rpy_list[2],
                "z" : position.z,
            }
            with open(f"{output_dir_head}{save_topic_dirs[topic]}/{filename}.json", 'w+') as json_file:
                json.dump(pose_dict, json_file, indent="\t")
        # gripper
        elif "gripper" in sensor_topic_dict and topic in sensor_topic_dict["gripper"]:
            msg = deserialize_message(data, get_message(topic_types_dict[topic]))
            filename = ros_time_to_sec_str(msg.header.stamp)
            angle = msg.angle
            distance = msg.distance
            gripper_dict = {
                "angle": angle,
                "distance": distance,
            }
            with open(f"{output_dir_head}{save_topic_dirs[topic]}/{filename}.json", 'w+') as json_file:
                json.dump(gripper_dict, json_file, indent="\t")

        # imu
        elif "imu" in sensor_topic_dict and topic in sensor_topic_dict["imu"]:
            msg = deserialize_message(data, get_message(topic_types_dict[topic]))
            filename = ros_time_to_sec_str(msg.header.stamp)
            imu_dict = {
                "angular_velocity" : {
                    "x" : msg.angular_velocity.x,
                    "y" : msg.angular_velocity.y,
                    "z" : msg.angular_velocity.z,
                },
                "linear_acceleration" : {
                    "x" : msg.linear_acceleration.x,
                    "y" : msg.linear_acceleration.y,
                    "z" : msg.linear_acceleration.z,
                },
                "orientation" : {
                    "x" : msg.orientation.x,
                    "y" : msg.orientation.y,
                    "z" : msg.orientation.z,
                    "w" : msg.orientation.w,
                },
            }
            with open(f"{output_dir_head}{save_topic_dirs[topic]}/{filename}.json", 'w+') as json_file:
                json.dump(imu_dict, json_file, indent="\t")

        # TODO /robotBase/transform
        # can't process
        elif topic not in fail_topics:
            fail_topics.append(topic)
    print()
    end_time = time.time()
    print("spend time: %-.3fs" % (end_time - start_time))

    if len(fail_topics) != 0:
        print("\n[Warn] can't generate aloha data from topic:")
        print("++++++++++++++++++++++++++++++++++++++++++")
        for topic in fail_topics:
            print(f"  * {topic}")
        print("------------------------------------------")

def get_mcap_files(in_dir=""):
    exclude_dirs = [".Trash-1000"]
    file_list = []
    for root, dirs, files in os.walk(in_dir):
        dirs[:] = [d for d in dirs if d not in exclude_dirs]
        
        for file in files:
            if file.endswith('.mcap'):
                file_path = os.path.join(root, file)
                file_list.append(file_path)
    return file_list

def main():
    mcap_dir = os.getcwd()
    data_dir = mcap_dir
    parser = argparse.ArgumentParser()
    parser.add_argument('--datasetDir', action='store', type=str, help='datasetDir.',
                        default=os.getcwd(), required=False)
    parser.add_argument('--episodeIndex', action='store', type=int, help='Episode index.',
                        default=-1, required=False)
    parser.add_argument('--alohaYaml', action='store', type=str, help='alohaYaml.',
                        default="/home/agilex/ros2_ws/src/data_tools/config/aloha_data_params.yaml", required=False)
    args = parser.parse_args()
    print("args:")
    print(f" --datasetDir: {args.datasetDir}")
    print(f" --episodeIndex: {args.episodeIndex}")
    print(f" --alohaYaml: {args.alohaYaml}")

    if args.datasetDir != "" and args.datasetDir != ".":
        mcap_dir = args.datasetDir
        data_dir = args.datasetDir
    if args.episodeIndex != -1:
        mcap_dir = os.path.join(mcap_dir, f"episode{args.episodeIndex}")

    # 获取指定目录下所有mcap
    files = get_mcap_files(mcap_dir)
    print(f"转换的mcaps: {files}")

    # 指定以某个yaml文件配置来将mcap转aloha数据
    for file in files:
        process_file(file, data_dir, args.alohaYaml)

if __name__ == "__main__":
    main()