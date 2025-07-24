SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
camera_fps=30
camera_width=640
camera_height=480
l_depth_camera_no=230322276296
r_depth_camera_no=230322273910

l_serial_port=/dev/ttyUSB60
r_serial_port=/dev/ttyUSB61
sudo chmod a+rw /dev/ttyUSB*
l_fisheye_port=60
r_fisheye_port=61
sudo chmod a+rw /dev/video*

source /opt/ros/noetic/setup.bash && cd $SCRIPT_DIR/../install/share/sensor_tools/scripts && chmod 777 usb_camera.py
if [ -n "$1" ]; then
    source $SCRIPT_DIR/../install/setup.bash && roslaunch sensor_tools open_multi_gripper.launch l_depth_camera_no:=$l_depth_camera_no r_depth_camera_no:=$r_depth_camera_no l_serial_port:=$l_serial_port r_serial_port:=$r_serial_port l_fisheye_port:=$l_fisheye_port r_fisheye_port:=$r_fisheye_port camera_fps:=$camera_fps camera_width:=$camera_width camera_height:=$camera_height name:=$1 name_index:=$1_
else
    source $SCRIPT_DIR/../install/setup.bash && roslaunch sensor_tools open_multi_gripper.launch l_depth_camera_no:=$l_depth_camera_no r_depth_camera_no:=$r_depth_camera_no l_serial_port:=$l_serial_port r_serial_port:=$r_serial_port l_fisheye_port:=$l_fisheye_port r_fisheye_port:=$r_fisheye_port camera_fps:=$camera_fps camera_width:=$camera_width camera_height:=$camera_height
fi
                