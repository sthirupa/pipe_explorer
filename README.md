# pipe_explorer
Perception, controls, planning, etc. stacks for the MRSD RoboSpect Pipe Autonomous Explorer Startup

## Install Dependencies
- [ROS2](https://docs.ros.org/) 
- [PIPE REPAIR GIT] (https://github.com/biorobotics/ARPA-E_pipe_repair_robot)

Before using ROS2 remember to source your terminal by: (Replace foxy with your ROS2 version)
```shell
source /opt/ros/foxy/setup.bash
```

## Build the Package
If you already have a catkin workspace skip this step, else
```shell
mkdir -p ~/ros2_ws/src
```

**Clone the repository and the build the package**
```shell
cd ~/ros2_ws/src
git clone https://github.com/sthirupa/pipe_explorer.git .
cd ..
colcon build --packages-select rs_hello_imshow piperobot_keycmds
. install/setup.bash
```

**Replace the teleop_node_dd file in the piperepair repository with the one in this git**
For example, if the teleop_node_dd.py file for the pipe repair repo was located at:

~/pipe_repair_ws/src/ARPA-E_pipe_repair_robot/motion_planning/motion_planning/teleop_node_dd.py

then run
```shell
mv ~/ros2_ws/src/teleop_node_dd.py ~/pipe_repair_ws/src/ARPA-E_pipe_repair_robot/motion_planning/motion_planning/teleop_node_dd.py
```

## Launch the ROS2 Nodes
Launch each of the following on different terminals (launch folder to come as soon as I figure that out)
```shell
cd ~/pipe_repair_ws && rosdep install --from-paths src --ignore-src -r -y && colcon build && source install/setup.bash && ros2 run joy joy_node
```

```shell
cd ~/pipe_repair_ws && rosdep install --from-paths src --ignore-src -r -y && colcon build && source install/setup.bash && ros2 run hebi_ros_driver hebi_ros_driver_node
```

```shell
cd ~/pipe_repair_ws && rosdep install --from-paths src --ignore-src -r -y && colcon build && source install/setup.bash && ros2 run motion_planning teleop_node_dd
```

To add the key commands from th piperobot_keycmds folder,

```shell
cd ~/ros2_ws && colcon build --packages-select piperobot_keycmds && . install/setup.bash && ros2 run piperobot_keycmds teleop_robot_node
```

To add realsense obstacle stop:
```shell
cd ~/ros2_ws && colcon build --packages-select rs_hello_imshow && sourcedev && ros2 run rs_hello_imshow node
```

```shell
cd ~/ros2_ws && colcon build --packages-select rs_hello_imshow && sourcedev && ros2 run rs_hello_imshow subscriber_node
```

## Onboard Computing actions
ssh to the onboard computing unit (the NUC) by using the command
```shell
ssh repair_robot@192.168.0.104
```
**but check the IP address in the event of changes**
password: biorobotics

## Acknowledgement
The repository was created to fulfill the need for a package to use a pipe repair robot for autonomous exploration 
The repository is a combined modification of following repositories:
- [realsense-ros](https://github.com/IntelRealSense/realsense-ros)
- [ARPA-E_pipe_repair_robot](https://github.com/biorobotics/ARPA-E_pipe_repair_robot)

