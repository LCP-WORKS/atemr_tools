#!/bin/bash

echo "delete remap the device serial port(ttyUSBX) to wt901IMU"
echo "sudo rm   /etc/udev/rules.d/wt901_imu.rules"
sudo rm   /etc/udev/rules.d/wt901_imu.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"
