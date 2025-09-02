l_serial_port=/dev/ttyUSB50
r_serial_port=/dev/ttyUSB51
sudo chmod a+rw /dev/ttyUSB*

if [ -n "$1" ]; then
    source ~/pika_ros/install/setup.bash && roslaunch sensor_tools open_multi_sensor.launch  l_serial_port:=$l_serial_port r_serial_port:=$r_serial_port  name:=$1 name_index:=$1_
else
    source ~/pika_ros/install/setup.bash && roslaunch sensor_tools open_multi_sensor.launch  l_serial_port:=$l_serial_port r_serial_port:=$r_serial_port 
fi
                