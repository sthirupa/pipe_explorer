# pipe_explorer
Perception, controls, planning, etc. stacks for the MRSD RoboSpect Pipe Autonomous Explorer Startup

## Install Dependencies
- [ROS2](https://docs.ros.org/) 

## Build the Package
If you already have a catkin workspace skip this step, else
```shell
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Before using ROS remember to source your terminal by: (Replace noetic with your ROS version)
```shell
source /opt/ros/foxy/setup.bash
```


**Clone the repository and the build the package**
```shell
git clone https://github.com/xujinyun/pipe_simulator.git .
cd ..
catkin_make
cakin_make install
```

If not done already, Insert this in your `~/.bashrc` file for sourcing your workspace:
```shell
source ~/catkin_ws/devel/setup.bash
```

## Launch the ROS-Package
The world name can be replaced with any self-defined world file stored in worlds folder.
```shell
roslaunch pipe_robot gazebo_with_sensor.launch world_name:=rust_pipe
```
To run linear moving control go to pipe_robot/src:
```shell
python3 linear_control_node.py 
```


## Acknowledgement
The repository was created to fulfill the need for a package to use simulated L515 sensor in Gazebo. 
The repository is a combined modification of following repositories:
- [realsense-ros](https://github.com/IntelRealSense/realsense-ros)
- [realsense_gazebo_plugin](https://github.com/pal-robotics/realsense_gazebo_plugin)
- [realsense-ros-gazebo](https://github.com/rickstaa/realsense-ros-gazebo/)
- [realsense-l515-gazebo-plugin](https://github.com/ZohebAbai/gazebo_ros_l515)

