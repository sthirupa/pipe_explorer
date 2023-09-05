import os
import launch

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    params = os.path.join(
        get_package_share_directory('ximea_usb_ros_driver'),
        'config',
        'multi_camera.yaml'
    )
    print('PARAMS', params)
    
    ld = launch.LaunchDescription()

    camera1_node = Node(
        package='ximea_usb_ros_driver',
        executable='ximea_usb_ros_driver_node',
        namespace='rgb',
        name='camera_1',
        parameters=[{
            'serial_number': 'UPCBS2110003',
            'frame_rate' : 2.0
            }]
    )
    ld.add_action(camera1_node)

    camera2_node = Node(
        package='ximea_usb_ros_driver',
        executable='ximea_usb_ros_driver_node',
        namespace='rgb',
        name='camera_2',
        parameters=[{
            'serial_number': 'UPCBS2110021',
            'frame_rate' : 2.0
            }]
    )
    ld.add_action(camera2_node)

    camera3_node = Node(
        package='ximea_usb_ros_driver',
        executable='ximea_usb_ros_driver_node',
        namespace='rgb',
        name='camera_3',
        parameters=[{
            'serial_number': 'UPCBS2031037',
            'frame_rate' : 2.0
            }]
    )
    ld.add_action(camera3_node)

    return ld