# sensor_tools

ros package sensor_tools, used to open the fisheye camera, retrieve topic /camera_fisheye/color/image_raw, gripper data topic /gripper/data and realsense data.

## Get Ready
## Installation
Ensure that the realsense SDK has been installed and place the realsense-ros package in the pika_ros/src directory.
Place this package and libsurvive in the workspace pika_ros/src.
```bash
cd ~/pika_ros
catkin_make
```

## Start
Ensure that your device is connected to the computer.
if you using conda, please activate your env
```bash
conda activate YOUR_ENV
```
and then
```bash
cd ~/pika_ros/src/sensor_tools/scripts/
bash start.bash
```
or
```bash
sudo sh -c 'echo "KERNEL==\"video*\", ATTRS{idVendor}==\"1bcf\", ATTRS{idProduct}==\"2cd1\", MODE:=\"0777\", SYMLINK+=\"video22\"" > /etc/udev/rules.d/fisheye.rules'
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger

sudo chmod a+rw /dev/ttyACM0
sudo chmod a+rw /dev/video22

chmod 777 usb_camera.py
source ~/pika_ros/devel/setup.bash && roslaunch sensor_tools open_sensor.launch
```