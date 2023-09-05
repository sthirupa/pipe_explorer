#!/bin/bash

# Bash file to setup the ARPA-E Pipe Repair robot motion drivers

echo Launching sensors.
WS_DIR='/home/arpa-e/pipe_crawler_ws/'

cd $WS_DIR
source install/setup.bash

ros2 launch perception sensors.launch.py
#ros2 launch data_compression data_compression.launch.py
