import launch
from launch_ros.actions import Node

def generate_launch_description():
    
    ld = launch.LaunchDescription()

    ximea_node = Node(
        package='ximea_usb_ros_driver',
        executable='ximea_usb_ros_driver_node',
        namespace='rgb',
        name='camera',
        parameters=[{
            'serial_number': 'UPCBS2110003',
            'frame_rate' : 1.0
            }]
    )
    ld.add_action(ximea_node)

    return ld