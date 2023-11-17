import hebi
import time
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
# from hebi_ros_driver.msg import HebiFeedback TODO currently only supported in C++


class HebiROS2Driver(Node):

    def __init__(self):
        super().__init__('hebi_driver')

        self.lookup = hebi.Lookup()
        time.sleep(1) # 1 sec
        print('Modules found on network:')
        for entry in self.lookup.entrylist:
            print(f'{entry.family} | {entry.name}')
        
        self.robot = self.lookup.get_group_from_names(['repair_robot'], ['right_motor', 'left_motor'])
        self.robot.command_lifetime = 500.0 # 0.5 secs
        self.robot.feedback_frequency = 100.0 # Hz

        self.robot_cmd = hebi.GroupCommand(self.robot.size)
        self.robot_fbk = hebi.GroupFeedback(self.robot.size)

        self.cmd_vel = np.array([0.0, 0.0])

        self._encoder_pub = self.create_publisher(Float64MultiArray, '/repair_robot/encoder', 1)
        self._joint_states_pub = self.create_publisher(JointState, '/repair_robot/joint_states', 1)
        # self._feedback_pub = self.create_publisher(HebiFeedback, '/repair_robot/feedback', 1)
        self._cmd_vel_sub = self.create_subscription(Float64MultiArray, '/repair_robot/cmd_vel', self._cmd_callback, 1)

        timer_period = 1/10 # secs
        self._timer = self.create_timer(timer_period, self._hebi_interface)

    def _cmd_callback(self, msg):

        self.cmd_vel = np.array(msg.data)

    def _hebi_interface(self):
        
        self.robot_cmd.velocity = self.cmd_vel
        self.robot.send_command(self.robot_cmd)

        fbk = self.robot_fbk
        self.robot_fbk = self.robot.get_next_feedback(reuse_fbk=self.robot_fbk)
        if self.robot_fbk is None:
            self.robot_fbk = fbk

        joint_states = JointState()
        joint_states.header.stamp = self.get_clock().now().to_msg()
        joint_states.name = ['right_motor', 'left_motor']
        joint_states.position = self.robot_fbk.position.tolist()
        joint_states.velocity = self.robot_fbk.velocity.tolist()
        joint_states.effort = self.robot_fbk.effort.tolist()

        self._joint_states_pub.publish(joint_states)

        # motor_fbk = HebiFeedback()
        # motor_fbk.header.stamp = self.get_clock().now().to_msg()
        # motor_fbk.name = ['right_motor', 'left_motor']
        # motor_fbk.current = self.robot_fbk.motor_current.tolist()
        # motor_fbk.voltage = self.robot_fbk.voltage.tolist()
        # motor_fbk.mstop_state = self.robot_fbk.mstop_state.tolist()
        # motor_fbk.temperature_state = self.robot_fbk.temperature_state.tolist()
        # motor_fbk.led = self.robot_fbk.led.color.tolist()
        # motor_fbk.board_temperature = self.robot_fbk.board_temperature.tolist()
        # motor_fbk.processor_temperature = self.robot_fbk.processor_temperature.tolist()
        # motor_fbk.motor_winding_temperature = self.robot_fbk.motor_winding_temperature.tolist()
        # self._feedback_pub.publish(motor_fbk)
        encoder = Float64MultiArray()
        encoder.data = joint_states.position
        self._encoder_pub.publish(encoder)
        print('position: ', self.robot_fbk.position.tolist())


def main():
    rclpy.init()
    node = HebiROS2Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == 'main':
    main()
