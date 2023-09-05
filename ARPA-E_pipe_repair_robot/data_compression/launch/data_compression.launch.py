from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    ld = LaunchDescription()

    """ Realsense Compression """
    compress_realsense = Node(
        package='data_compression',
        executable='compress_realsense'
    )
    ld.add_action(compress_realsense)

    """ Ximea RGB Compression """
    compress_rgb = Node(
        package='data_compression',
        executable='compress_rgb'
    )
    ld.add_action(compress_rgb)

    """ Thermal Compression """
    compress_thermal = Node(
        package='data_compression',
        executable='compress_thermal'
    )
    ld.add_action(compress_thermal)

    return ld