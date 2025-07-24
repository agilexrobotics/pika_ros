SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
camera_fps=30
camera_width=640
camera_height=480
sudo sh -c 'echo "KERNEL==\"ttyUSB*\", ATTRS{idVendor}==\"1a86\", ATTRS{idProduct}==\"7522\", MODE:=\"0777\", SYMLINK+=\"ttyUSB60\"" > /etc/udev/rules.d/gripper_serial.rules'
sudo sh -c 'echo "KERNEL==\"video*\", ATTRS{idVendor}==\"1bcf\", ATTRS{idProduct}==\"2cd1\", MODE:=\"0777\", SYMLINK+=\"video60\"" > /etc/udev/rules.d/gripper_fisheye.rules'
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger

sudo chmod a+rw /dev/ttyUSB*
sudo chmod a+rw /dev/video*

source /opt/ros/noetic/setup.bash && cd $SCRIPT_DIR/../install/share/sensor_tools/scripts && chmod 777 usb_camera.py
source $SCRIPT_DIR/../install/setup.bash && roslaunch sensor_tools open_single_gripper.launch serial_port:=/dev/ttyUSB60 fisheye_port:=60 camera_fps:=$camera_fps camera_width:=$camera_width camera_height:=$camera_height
