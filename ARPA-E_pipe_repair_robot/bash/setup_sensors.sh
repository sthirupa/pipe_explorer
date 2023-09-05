#!/bin/bash

# Bash file to setup the ARPA-E Pipe Repair robot sensor payload drivers

echo Starting sensor payload setup.
REPO_DIR='/home/arpa-e/pipe_crawler_ws/src/ARPA-E_pipe_repair'
PASSWORD=$(<~/.password)

# Set up Intel Realsense L515 Depth Camera

# Set up XIMEA MU181CR-ON RGB Cameras
echo $PASSWORD | sudo -S tee /sys/module/usbcore/parameters/usbfs_memory_mb >/dev/null <<<0

# Set up Lepton Thermal Cameras
echo $PASSWORD | sudo -S modprobe -r uvcvideo
echo $PASSWORD | sudo -S modprobe uvcvideo quirks=0x002 nodrop=1 timeout=5000

echo Sensor payload setup complete.