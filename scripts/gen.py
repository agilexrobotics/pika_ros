#!/usr/bin/env python3

import subprocess
import re
import os
import cv2

def run_command(command):
    """运行命令并返回输出"""
    try:
        result = subprocess.run(command, shell=True, capture_output=True, text=True)
        return result.stdout.strip()
    except Exception as e:
        print(f"执行命令时出错: {str(e)}")
        return None


def get_device_info():
    """获取设备信息"""
    # 运行 rs-enumerate-devices 命令
    rs_output = run_command("rs-enumerate-devices -s")
    if not rs_output:
        return None, None

    # 解析输出获取序列号
    serial_match = re.search(r'Intel RealSense D405\s+(\d+)', rs_output)
    if not serial_match:
        return None, None
    serial_number = serial_match.group(1)

    # 运行 udevadm 命令
    udev_output = run_command("udevadm info /dev/ttyUSB0 | grep DEVPATH")
    if not udev_output:
        return None, None

    # 解析 USB 路径
    usb_path = udev_output[:udev_output.find("ttyUSB0")][:-1]  # 获取 1-13.2.4:1.0 这样的格式
    usb_path = usb_path[usb_path.rfind("/")+1:]
    print("\n寻找鱼眼摄像头，请在出现鱼眼摄像头时按下s，非鱼眼摄像头则按下q")
    video_path = None
    for i in range(50):
        cap = cv2.VideoCapture(i)
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        key = None
        if cap.isOpened():
            print("port:", "/dev/video"+str(i))
            while True:
                ret, frame = cap.read()
                cv2.imshow("/dev/video"+str(i), frame)
                key = cv2.waitKey(1)
                if key & 0xFF == ord('q'):
                    break
                elif key & 0xFF == ord('s'):
                    break
        if key is not None and key & 0xFF == ord('s'):
            video_path = 'video' + str(i)
            break
    cv2.destroyAllWindows()
    if video_path is None:
        return None, None
    print(video_path)
    udev_output = run_command(f"udevadm info /dev/{video_path} | grep DEVPATH")
    print(udev_output)
    video_path = udev_output[:udev_output.find("video")][:-1]  # 获取 1-13.2.4:1.0 这样的格式
    print(video_path)
    video_path = video_path[video_path.rfind("/")+1:]

    return serial_number, usb_path, video_path


def generate_setup_bash(left_info, right_info):
    """生成 setup.bash 文件"""
    content = f"""
#/bin/bash

sudo sh -c 'echo "ACTION==\\"add\\", KERNELS==\\"{left_info[1]}\\", SUBSYSTEMS==\\"usb\\", MODE:=\\"0777\\", SYMLINK+=\\"ttyUSB22\\"" > /etc/udev/rules.d/serial.rules'
sudo sh -c 'echo "ACTION==\\"add\\", KERNELS==\\"{right_info[1]}\\", SUBSYSTEMS==\\"usb\\", MODE:=\\"0777\\", SYMLINK+=\\"ttyUSB23\\"" >> /etc/udev/rules.d/serial.rules'

sudo sh -c 'echo "ACTION==\\"add\\", KERNEL==\\"video[0,2,4,6,8,10,12,14,16,18,20]*\\", KERNELS==\\"{left_info[2]}\\", SUBSYSTEMS==\\"usb\\", MODE:=\\"0777\\", SYMLINK+=\\"video22\\"" > /etc/udev/rules.d/fisheye.rules'
sudo sh -c 'echo "ACTION==\\"add\\", KERNEL==\\"video[0,2,4,6,8,10,12,14,16,18,20]*\\", KERNELS==\\"{right_info[2]}\\", SUBSYSTEMS==\\"usb\\", MODE:=\\"0777\\", SYMLINK+=\\"video23\\"" >> /etc/udev/rules.d/fisheye.rules'

#sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
               """
    with open("setup.bash", "w") as f:
        f.write(content)
    os.chmod("setup.bash", 0o755)


def generate_start_multi_sensor_bash(left_info, right_info):
    """生成 start_multi_sensor.bash 文件"""
    content = f"""
camera_fps=30
camera_width=640
camera_height=480
l_depth_camera_no={left_info[0]}
r_depth_camera_no={right_info[0]}

l_serial_port=/dev/ttyUSB22
r_serial_port=/dev/ttyUSB23
sudo chmod a+rw $l_serial_port
sudo chmod a+rw $r_serial_port
l_fisheye_port=22
r_fisheye_port=23
sudo chmod a+rw /dev/video$l_fisheye_port
sudo chmod a+rw /dev/video$r_fisheye_port

source /opt/ros/noetic/setup.bash && cd ~/pika_ros/install/share/sensor_tools/scripts && chmod 777 usb_camera.py
source ~/pika_ros/install/setup.bash && roslaunch sensor_tools open_multi_sensor.launch l_depth_camera_no:=$l_depth_camera_no r_depth_camera_no:=$r_depth_camera_no l_serial_port:=$l_serial_port r_serial_port:=$r_serial_port l_fisheye_port:=$l_fisheye_port r_fisheye_port:=$r_fisheye_port camera_fps:=$camera_fps camera_width:=$camera_width camera_height:=$camera_height
               """

    with open("start_multi_sensor.bash", "w") as f:
        f.write(content)
    os.chmod("start_multi_sensor.bash", 0o755)


def main():
    print("=== RealSense 相机配置工具 ===")

    # 获取左侧相机信息
    print("\n请插入左设备，然后按回车键继续...")
    input()
    print("正在获取左设备信息...")
    left_info = get_device_info()
    if not left_info[0]:
        print("无法获取左侧相机信息，请检查设备连接")
        return
    print(f"左侧相机信息: {left_info[0]} {left_info[1]} {left_info[2]}")

    # 获取右侧相机信息
    print("\n请拔出左设备，插入右设备（注意不要插在同一个USB口，配置完成后USB口不能改变），然后按回车键继续...")
    input()
    print("正在获取右设备信息...")
    right_info = get_device_info()
    if not right_info[0]:
        print("无法获取右设备信息，请检查设备连接")
        return
    print(f"右设备信息: {right_info[0]} {right_info[1]} {right_info[2]}")

    # 生成配置文件
    print("\n正在生成配置文件...")
    generate_setup_bash(left_info, right_info)
    generate_start_multi_sensor_bash(left_info, right_info)

    print("\n配置完成！已生成以下文件：")
    print("1. setup.bash")
    print("2. start_multi_sensor.bash")
    print("\n执行setup.bash")
    run_command("bash setup.bash")
    print("\n执行完成，请拔插设备，注意插入先前绑定的同一个USB口。启动设备方法：")
    print("\n启动设备方法：")
    print("2. 然后运行: bash start_multi_sensor.bash")


if __name__ == "__main__":
    main()
