#!/bin/bash

# Bash file to setup the ARPA-E Pipe Repair robot motion drivers

echo Starting robot setup.
WS_DIR='/home/arpa-e/pipe_crawler_ws/'

cd $WS_DIR
source install/setup.bash

ros2 launch motion_planning teleop_dd.launch.py
