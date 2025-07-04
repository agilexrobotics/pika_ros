
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
camera_fps=30
camera_width=640
camera_height=480
l_depth_camera_no=230322272110
r_depth_camera_no=230322270988

l_serial_port=/dev/ttyUSB50
r_serial_port=/dev/ttyUSB51
sudo chmod a+rw /dev/ttyUSB*
l_fisheye_port=50
r_fisheye_port=51
sudo chmod a+rw /dev/video*

source /opt/ros/humble/setup.bash && cd $SCRIPT_DIR/../install/sensor_tools/share/sensor_tools/scripts/ && chmod 777 usb_camera.py
if [ -n "$1" ]; then
    source $SCRIPT_DIR/../install/setup.bash && ros2 launch sensor_tools open_multi_sensor.launch.py l_depth_camera_no:=_$l_depth_camera_no r_depth_camera_no:=_$r_depth_camera_no l_serial_port:=$l_serial_port r_serial_port:=$r_serial_port l_fisheye_port:=$l_fisheye_port r_fisheye_port:=$r_fisheye_port camera_fps:=$camera_fps camera_width:=$camera_width camera_height:=$camera_height camera_profile:=$camera_width,$camera_height,$camera_fps name:=$1 name_index:=$1_
else
    source $SCRIPT_DIR/../install/setup.bash && ros2 launch sensor_tools open_multi_sensor.launch.py l_depth_camera_no:=_$l_depth_camera_no r_depth_camera_no:=_$r_depth_camera_no l_serial_port:=$l_serial_port r_serial_port:=$r_serial_port l_fisheye_port:=$l_fisheye_port r_fisheye_port:=$r_fisheye_port camera_fps:=$camera_fps camera_width:=$camera_width camera_height:=$camera_height camera_profile:=$camera_width,$camera_height,$camera_fps
fi
                