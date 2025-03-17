camera_fps=30
camera_width=640
camera_height=480
l_depth_camera_no=230322275620
r_depth_camera_no=230322274185

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
