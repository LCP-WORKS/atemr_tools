#!/bin/bash

echo "remap the device serial port(ttyUSBX) to wt901IMU"
echo "wt901_imu usb connection as /dev/wt901IMU , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy wt901_imu.rules to  /etc/udev/rules.d/"
echo "`rospack find wt901_imu`/scripts/wt901_imu.rules"
sudo cp `rospack find wt901_imu`/scripts/wt901_imu.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "
