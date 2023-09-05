import sys
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    ld = LaunchDescription()

    """ Thermal Camera """
    thermal_node = Node(
        package='flir_lepton_ros_driver',
        executable='flir_lepton_ros_driver_node'
    )
    ld.add_action(thermal_node)

    """ XIMEA single camera """
    # TODO: very dirty solution, make this elegant
    # single_rgb_cam_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(['/home/arpa-e/pipe_crawler_ws/src/ARPA-E_pipe_repair_robot/drivers/ximea_usb_ros_driver/launch/single_camera.launch.py'])
    # )
    # ld.add_action(single_rgb_cam_launch)

    """ XIMEA multi camera """
    # TODO: very dirty solution, make this elegant
    rgb_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(['/home/arpa-e/pipe_crawler_ws/src/ARPA-E_pipe_repair_robot/drivers/ximea_usb_ros_driver/launch/multi_camera.launch.py'])
    )
    ld.add_action(rgb_cam_launch)

    """ Realsense Launch """
    # TODO: very dirty solution, make this elegant
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(['/home/arpa-e/pipe_crawler_ws/src/ARPA-E_pipe_repair_robot/drivers/realsense-ros/realsense2_camera/launch/rs_launch.py'])
    )
    ld.add_action(realsense_launch)

    """ Data Compression Launch """
    # TODO: very dirty solution, make this elegant
    data_compression_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(['/home/arpa-e/pipe_crawler_ws/src/ARPA-E_pipe_repair_robot/data_compression/launch/data_compression.launch.py'])
    )
    ld.add_action(data_compression_launch)

    """ Image Stitcher """
    stitcher_node = Node(
        package='perception',
        executable='image_stitcher'
    )
    ld.add_action(stitcher_node)
    
    return ld