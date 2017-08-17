#!/bin/bash

echo "***************"
echo "remap the device serial ports(ttyUSBX) to wpv4 device"
echo "start copy wpv4.rules to  /etc/udev/rules.d/"
sudo cp `rospack find wpv4_bringup`/scripts/wpv4.rules  /etc/udev/rules.d
echo "Restarting udev"
sudo service udev reload
sudo service udev restart
echo "finish"
echo "***************"
