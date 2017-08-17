#!/bin/bash

echo "***************"
echo "delete the remap device serial ports of wpv4"
sudo rm   /etc/udev/rules.d/wpv4.rules
echo "Restarting udev"
sudo service udev reload
sudo service udev restart
echo "finish"
echo "***************"
