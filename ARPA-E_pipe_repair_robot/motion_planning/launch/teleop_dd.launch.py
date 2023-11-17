from launch import LaunchDescription
from launch_ros.actions import Node

def _check_joystick():
    # TODO Check if something like this is possible or not
    return False

def generate_launch_description():

    ld = LaunchDescription()
    flag = _check_joystick()

    # driver_node = Node(
    #     package='copley_ros_driver',
    #     executable='copley_ros_driver_node'
    # )
    driver_node = Node(
        package='hebi_ros_driver',
        executable='hebi_ros_driver_node'
    )
    teleop_node_dd = Node(
        package='motion_planning',
        executable='teleop_node_dd'
    )

    imu_extract_node = Node(
        package='motion_planning',
        executable='imu_extractor'
    )

    if flag: 
        print('launching joystick')
        joystick_node = Node(
            package='joy',
            executable='joy_node'
        )
        ld.add_action(joystick_node)

    ld.add_action(driver_node)
    ld.add_action(teleop_node_dd)
    ld.add_action(imu_extract_node)
    
    return ld

