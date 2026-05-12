SCRIPT_DIR=$(dirname "$(readlink -f "$0")")

sudo sh -c 'echo "KERNEL==\"ttyUSB*\", ATTRS{idVendor}==\"1a86\", ATTRS{idProduct}==\"7522\", MODE:=\"0777\", SYMLINK+=\"ttyUSB50\"" > /etc/udev/rules.d/sensor_serial.rules'
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger

sudo chmod a+rw /dev/ttyUSB*

# source /opt/ros/humble/setup.bash && cd $SCRIPT_DIR/../install/sensor_tools/share/sensor_tools/scripts/ && chmod 777 usb_camera.py
source $SCRIPT_DIR/../install/setup.bash && ros2 launch sensor_tools open_single_sensor_with_teleop.launch.py serial_port:=/dev/ttyUSB50 joint_name:=center_joint

