
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
camera_fps=30
camera_width=640
camera_height=480
sensor_depth_camera_no=230322270606
gripper_depth_camera_no=230322274868

sensor_serial_port=/dev/ttyUSB50
gripper_serial_port=/dev/ttyUSB60
sudo chmod a+rw /dev/ttyUSB*
sensor_fisheye_port=50
gripper_fisheye_port=60
sudo chmod a+rw /dev/video*

source /opt/ros/humble/setup.bash && cd $SCRIPT_DIR/../install/sensor_tools/share/sensor_tools/scripts/ && chmod 777 usb_camera.py
source $SCRIPT_DIR/../install/setup.bash && ros2 launch sensor_tools open_sensor_gripper.launch.py sensor_depth_camera_no:=_$sensor_depth_camera_no gripper_depth_camera_no:=_$gripper_depth_camera_no sensor_serial_port:=$sensor_serial_port gripper_serial_port:=$gripper_serial_port sensor_fisheye_port:=$sensor_fisheye_port gripper_fisheye_port:=$gripper_fisheye_port camera_fps:=$camera_fps camera_width:=$camera_width camera_height:=$camera_height camera_profile:=$camera_width,$camera_height,$camera_fps
                