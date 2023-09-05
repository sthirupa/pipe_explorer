#!/bin/bash

# Bash file to setup the ARPA-E Pipe Repair robot motion drivers

echo Starting robot setup.
REPO_DIR='/home/arpa-e/pipe_crawler_ws/src/ARPA-E_pipe_repair_robot'
PASSWORD=$(<~/.password)

# Provide device permissions for arduino
echo $PASSWORD | sudo -S chmod 666 /dev/ttyACM0
echo $PASSWORD | sudo -S chmod 666 /dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_558383235353514112B2-if00

# Source ROS workspace
cd $REPO_DIR/../..
source install/setup.bash

echo Robot setup complete.
