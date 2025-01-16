#!/bin/bash

#udevadm info /dev/video22

sudo sh -c 'echo "ACTION==\"add\", KERNELS==\"1-12.4:1.0\", SUBSYSTEMS==\"usb\", MODE:=\"0777\", SYMLINK+=\"ttyUSB22\"" > /etc/udev/rules.d/serial.rules'
sudo sh -c 'echo "ACTION==\"add\", KERNELS==\"1-13.4:1.0\", SUBSYSTEMS==\"usb\", MODE:=\"0777\", SYMLINK+=\"ttyUSB23\"" >> /etc/udev/rules.d/serial.rules'

sudo sh -c 'echo "ACTION==\"add\", KERNEL==\"video[0,2,4,6,8,10,12,14,16,18,20]*\", KERNELS==\"1-12.3:1.0\", SUBSYSTEMS==\"usb\", MODE:=\"0777\", SYMLINK+=\"video22\"" > /etc/udev/rules.d/fisheye.rules'
sudo sh -c 'echo "ACTION==\"add\", KERNEL==\"video[0,2,4,6,8,10,12,14,16,18,20]*\", KERNELS==\"1-13.3:1.0\", SUBSYSTEMS==\"usb\", MODE:=\"0777\", SYMLINK+=\"video23\"" >> /etc/udev/rules.d/fisheye.rules'

sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
