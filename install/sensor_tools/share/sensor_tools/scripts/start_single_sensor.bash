SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
camera_fps=30
camera_width=640
camera_height=480
sudo sh -c 'echo "KERNEL==\"ttyUSB*\", ATTRS{idVendor}==\"1a86\", ATTRS{idProduct}==\"7522\", MODE:=\"0777\", SYMLINK+=\"ttyUSB50\"" > /etc/udev/rules.d/sensor_serial.rules'
sudo sh -c 'echo "KERNEL==\"video*\", ATTRS{idVendor}==\"1bcf\", ATTRS{idProduct}==\"2cd1\", MODE:=\"0777\", SYMLINK+=\"video50\"" > /etc/udev/rules.d/sensor_fisheye.rules'
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger

sudo chmod a+rw /dev/ttyUSB*
sudo chmod a+rw /dev/video*

source /opt/ros/humble/setup.bash && cd $SCRIPT_DIR/../install/sensor_tools/share/sensor_tools/scripts/ && chmod 777 usb_camera.py
source $SCRIPT_DIR/../install/setup.bash && ros2 launch sensor_tools open_single_sensor.launch.py serial_port:=/dev/ttyUSB50 fisheye_port:=50 camera_fps:=$camera_fps camera_width:=$camera_width camera_height:=$camera_height camera_profile:=$camera_width,$camera_height,$camera_fps joint_name:=center_joint

