#!/bin/bash

camera_fps=30
camera_width=640
camera_height=480
sudo sh -c 'echo "KERNEL==\"video*\", ATTRS{idVendor}==\"1bcf\", ATTRS{idProduct}==\"2cd1\", MODE:=\"0777\", SYMLINK+=\"video22\"" > /etc/udev/rules.d/fisheye.rules'
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger

sudo chmod a+rw /dev/ttyUSB0
sudo chmod a+rw /dev/video22

source /opt/ros/noetic/setup.bash && cd ~/pika_ws/install/share/sensor_tools/scripts && chmod 777 usb_camera.py
source ~/pika_ws/install/setup.bash && roslaunch sensor_tools open_sensor.launch serial_port:=/dev/ttyUSB0 camera_fps:=$camera_fps camera_width:=$camera_width camera_height:=$camera_height
