#!/bin/bash
sudo modprobe ftdi_sio
sleep 3s

su - root -c 'echo "165C 0009" > /sys/bus/usb-serial/drivers/ftdi_sio/new_id'
sleep 3s

sudo chmod a+rw /dev/sensors/ttyUSBRS485
sleep 3s

echo "Finish!!"

roslaunch fixed_horn home_neck.launch
