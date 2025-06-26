#!/usr/bin/env python3

import subprocess
import re
import os
import cv2
import time

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
        print("无法获取到深度摄像头数据")
        return None, None

    # 解析输出获取序列号
    serial_match = re.search(r'Intel RealSense D405\s+(\d+)', rs_output)
    if not serial_match:
        print("无法获取到深度摄像头数据")
        return None, None
    serial_number = serial_match.group(1)

    # 运行 udevadm 命令
    ls_output = run_command("ls /dev | grep ttyUSB | grep -v ttyUSB50 | grep -v ttyUSB51 | grep -v ttyUSB60 | grep -v ttyUSB61")
    count = ls_output.count("tty")
    if count > 1:
        print("请确保工控机只插入一个USB串口设备")
        return None, None
    udev_output = run_command(f"udevadm info /dev/{ls_output} | grep DEVPATH")
    if not udev_output:
        print("无法获取到串口数据")
        return None, None

    # 解析 USB 路径
    usb_path = udev_output[:udev_output.find(ls_output)][:-1]  # 获取 1-13.2.4:1.0 这样的格式
    usb_path = usb_path[usb_path.rfind("/")+1:]
    print("寻找鱼眼摄像头，请在出现鱼眼摄像头时按下s，非鱼眼摄像头则按下q(注意在图像窗口按下，不要在终端！！！)")
    video_path = None
    cv2.setLogLevel(0)
    for i in range(50):
        cap = cv2.VideoCapture(i)
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        key = None
        if cap.isOpened():
            # print("port:", "/dev/video"+str(i))
            while True:
                ret, frame = cap.read()
                cv2.imshow("/dev/video"+str(i), frame)
                key = cv2.waitKey(1)
                if key & 0xFF == ord('q'):
                    break
                elif key & 0xFF == ord('s'):
                    break
        cv2.destroyAllWindows()
        if key is not None and key & 0xFF == ord('s'):
            video_path = 'video' + str(i)
            break
    cv2.destroyAllWindows()
    if video_path is None:
        print("无法获取到鱼眼摄像头数据")
        return None, None
    udev_output = run_command(f"udevadm info /dev/{video_path} | grep DEVPATH")
    video_path = udev_output[:udev_output.find("video")][:-1]  # 获取 1-13.2.4:1.0 这样的格式
    video_path = video_path[video_path.rfind("/")+1:]

    return serial_number, usb_path, video_path


def generate_setup_bash(left_info, right_info, select):
    if select == "1":
        path = "setup_multi_sensor.bash"
        usb_num1 = 50
        usb_num2 = 51
        name1 = "sensor_"
        name2 = "sensor_"
        to1 = ">"
        to2 = ">>"
    if select == "2":
        path = "setup_multi_gripper.bash"
        usb_num1 = 60
        usb_num2 = 61
        name1 = "gripper_"
        name2 = "gripper_"
        to1 = ">"
        to2 = ">>"
    if select == "3":
        path = "setup_sensor_gripper.bash"
        usb_num1 = 50
        usb_num2 = 60
        name1 = "sensor_"
        name2 = "gripper_"
        to1 = ">"
        to2 = ">"
    """生成 setup.bash 文件"""
    content = f"""
#/bin/bash

sudo sh -c 'echo "ACTION==\\"add\\", KERNELS==\\"{left_info[1]}\\", SUBSYSTEMS==\\"usb\\", MODE:=\\"0777\\", SYMLINK+=\\"ttyUSB{usb_num1}\\"" {to1} /etc/udev/rules.d/{name1}serial.rules'
sudo sh -c 'echo "ACTION==\\"add\\", KERNELS==\\"{right_info[1]}\\", SUBSYSTEMS==\\"usb\\", MODE:=\\"0777\\", SYMLINK+=\\"ttyUSB{usb_num2}\\"" {to2} /etc/udev/rules.d/{name2}serial.rules'

sudo sh -c 'echo "ACTION==\\"add\\", KERNEL==\\"video[0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48]*\\", KERNELS==\\"{left_info[2]}\\", SUBSYSTEMS==\\"usb\\", MODE:=\\"0777\\", SYMLINK+=\\"video{usb_num1}\\"" {to1} /etc/udev/rules.d/{name1}fisheye.rules'
sudo sh -c 'echo "ACTION==\\"add\\", KERNEL==\\"video[0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48]*\\", KERNELS==\\"{right_info[2]}\\", SUBSYSTEMS==\\"usb\\", MODE:=\\"0777\\", SYMLINK+=\\"video{usb_num2}\\"" {to2} /etc/udev/rules.d/{name2}fisheye.rules'

sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
               """
    with open(path, "w") as f:
        f.write(content)
    os.chmod(path, 0o755)


def generate_start_bash(left_info, right_info, select):
    if select == "1":
        path = "start_multi_sensor.bash"
        usb_num1 = 50
        usb_num2 = 51
        content = f"""
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
camera_fps=30
camera_width=640
camera_height=480
l_depth_camera_no={left_info[0]}
r_depth_camera_no={right_info[0]}

l_serial_port=/dev/ttyUSB{usb_num1}
r_serial_port=/dev/ttyUSB{usb_num2}
sudo chmod a+rw /dev/ttyUSB*
l_fisheye_port={usb_num1}
r_fisheye_port={usb_num2}
sudo chmod a+rw /dev/video*

source /opt/ros/humble/setup.bash && cd $SCRIPT_DIR/../install/sensor_tools/share/sensor_tools/scripts/ && chmod 777 usb_camera.py
if [ -n "$1" ]; then
    source $SCRIPT_DIR/../install/setup.bash && ros2 launch sensor_tools open_multi_sensor.launch.py l_depth_camera_no:=_$l_depth_camera_no r_depth_camera_no:=_$r_depth_camera_no l_serial_port:=$l_serial_port r_serial_port:=$r_serial_port l_fisheye_port:=$l_fisheye_port r_fisheye_port:=$r_fisheye_port camera_fps:=$camera_fps camera_width:=$camera_width camera_height:=$camera_height camera_profile:=$camera_width,$camera_height,$camera_fps name:=$1 name_index:=$1_
else
    source $SCRIPT_DIR/../install/setup.bash && ros2 launch sensor_tools open_multi_sensor.launch.py l_depth_camera_no:=_$l_depth_camera_no r_depth_camera_no:=_$r_depth_camera_no l_serial_port:=$l_serial_port r_serial_port:=$r_serial_port l_fisheye_port:=$l_fisheye_port r_fisheye_port:=$r_fisheye_port camera_fps:=$camera_fps camera_width:=$camera_width camera_height:=$camera_height camera_profile:=$camera_width,$camera_height,$camera_fps
fi
                """
    if select == "2":
        path = "start_multi_gripper.bash"
        usb_num1 = 60
        usb_num2 = 61
        content = f"""
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
camera_fps=30
camera_width=640
camera_height=480
l_depth_camera_no={left_info[0]}
r_depth_camera_no={right_info[0]}

l_serial_port=/dev/ttyUSB{usb_num1}
r_serial_port=/dev/ttyUSB{usb_num2}
sudo chmod a+rw /dev/ttyUSB*
l_fisheye_port={usb_num1}
r_fisheye_port={usb_num2}
sudo chmod a+rw /dev/video*

source /opt/ros/humble/setup.bash && cd $SCRIPT_DIR/../install/sensor_tools/share/sensor_tools/scripts/ && chmod 777 usb_camera.py
if [ -n "$1" ]; then
    source $SCRIPT_DIR/../install/setup.bash && ros2 launch sensor_tools open_multi_gripper.launch.py l_depth_camera_no:=_$l_depth_camera_no r_depth_camera_no:=_$r_depth_camera_no l_serial_port:=$l_serial_port r_serial_port:=$r_serial_port l_fisheye_port:=$l_fisheye_port r_fisheye_port:=$r_fisheye_port camera_fps:=$camera_fps camera_width:=$camera_width camera_height:=$camera_height camera_profile:=$camera_width,$camera_height,$camera_fps name:=$1 name_index:=$1_
else
    source $SCRIPT_DIR/../install/setup.bash && ros2 launch sensor_tools open_multi_gripper.launch.py l_depth_camera_no:=_$l_depth_camera_no r_depth_camera_no:=_$r_depth_camera_no l_serial_port:=$l_serial_port r_serial_port:=$r_serial_port l_fisheye_port:=$l_fisheye_port r_fisheye_port:=$r_fisheye_port camera_fps:=$camera_fps camera_width:=$camera_width camera_height:=$camera_height camera_profile:=$camera_width,$camera_height,$camera_fps
fi
                """
    if select == "3":
        path = "start_sensor_gripper.bash"
        usb_num1 = 50
        usb_num2 = 60
        content = f"""
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
camera_fps=30
camera_width=640
camera_height=480
sensor_depth_camera_no={left_info[0]}
gripper_depth_camera_no={right_info[0]}

sensor_serial_port=/dev/ttyUSB{usb_num1}
gripper_serial_port=/dev/ttyUSB{usb_num2}
sudo chmod a+rw /dev/ttyUSB*
sensor_fisheye_port={usb_num1}
gripper_fisheye_port={usb_num2}
sudo chmod a+rw /dev/video*

source /opt/ros/humble/setup.bash && cd $SCRIPT_DIR/../install/sensor_tools/share/sensor_tools/scripts/ && chmod 777 usb_camera.py
source $SCRIPT_DIR/../install/setup.bash && ros2 launch sensor_tools open_sensor_gripper.launch.py sensor_depth_camera_no:=_$sensor_depth_camera_no gripper_depth_camera_no:=_$gripper_depth_camera_no sensor_serial_port:=$sensor_serial_port gripper_serial_port:=$gripper_serial_port sensor_fisheye_port:=$sensor_fisheye_port gripper_fisheye_port:=$gripper_fisheye_port camera_fps:=$camera_fps camera_width:=$camera_width camera_height:=$camera_height camera_profile:=$camera_width,$camera_height,$camera_fps
                """
    with open(path, "w") as f:
        f.write(content)
    os.chmod(path, 0o755)


def main():
    print("=== pika配置工具 ===")
    select = None
    while True:
        select = input("请选择绑定\n1.两个pika sensor(手持夹爪)\n2.两个pika gripper(安装于机械臂上的夹爪)\n3.一个pika sensor 一个pika gripper\n请输入：")
        if select == "1":
            device1 = "左"
            device2 = "右"
            break
        if select == "2":
            device1 = "左"
            device2 = "右"
            break
        if select == "3":
            device1 = "sensor"
            device2 = "gripper"
            break
        else:
            print("请输入1、2或3")
            continue

    print(f"请插入{device1}设备，然后按回车键继续...")
    input()
    print(f"正在获取{device1}设备信息...")
    while True:
        left_info = get_device_info()
        if not left_info[0]:
            print(f"无法获取{device1}设备信息，请检查设备连接，然后按回车键继续...")
            input()
        else:
            break
    print(f"{device1}设备信息: {left_info[0]} {left_info[1]} {left_info[2]}")


    print(f"请拔出{device1}设备，插入{device2}设备（注意不要插在同一个USB口，配置完成后USB口不能改变），然后按回车键继续...")
    input()
    print(f"正在获取{device2}设备信息...")
    while True:
        right_info = get_device_info()
        if not right_info[0]:
            print(f"无法获取{device2}设备信息，请检查设备连接，然后按回车键继续...")
            input()
        else:
            break
    print(f"{device2}设备信息: {right_info[0]} {right_info[1]} {right_info[2]}")

    # 生成配置文件
    print("正在生成配置文件...")
    generate_setup_bash(left_info, right_info, select)
    generate_start_bash(left_info, right_info, select)
    setup_path = "setup_multi_sensor.bash" if select=="1" else ("setup_multi_gripper.bash" if select=="2" else "setup_sensor_gripper.bash")
    start_path = "start_multi_sensor.bash" if select=="1" else ("start_multi_gripper.bash" if select=="2" else "start_sensor_gripper.bash")
    print("配置完成！已生成以下文件：")
    print(f"1. {setup_path}")
    print(f"2. {start_path}")
    print(f"执行{setup_path}")
    run_command(f"bash {setup_path}")
    print("执行完成。")
    while True:
        print("请拔插设备，注意插入先前绑定的同一个USB口。然后按回车键检查是否绑定成功...")
        input()
        print("请等待...")
        time.sleep(5)
        video_list = run_command("ls /dev | grep video")
        usb_list = run_command("ls /dev | grep ttyUSB")
        if (select == "1" or select == "3") and video_list.find("50") < 0:
            print("找不到sensor（左）鱼眼")
            continue
        if (select == "1") and video_list.find("51") < 0:
            print("找不到sensor（右）鱼眼")
            continue
        if (select == "2" or select == "3") and video_list.find("60") < 0:
            print("找不到gripper（左）鱼眼")
            continue
        if (select == "2") and video_list.find("61") < 0:
            print("找不到gripper（右）鱼眼")
            continue
        if (select == "1" or select == "3") and usb_list.find("50") < 0:
            print("找不到sensor（左）串口")
            continue
        if (select == "1") and usb_list.find("51") < 0:
            print("找不到sensor（右）串口")
            continue
        if (select == "2" or select == "3") and usb_list.find("60") < 0:
            print("找不到gripper（左）串口")
            continue
        if (select == "2") and usb_list.find("61") < 0:
            print("找不到gripper（右）串口")
            continue
        break
    print("绑定成功，启动设备方法：")
    print(f"2. 然后运行: bash {start_path}")


if __name__ == "__main__":
    main()
