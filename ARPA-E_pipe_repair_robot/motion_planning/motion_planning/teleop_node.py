import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
# from std_msgs.msg import String


class JoystickController(Node):
    """ Node to control the ARPA-E Pipe Repair robot with a joystick.
    
    Controls:
        RT - Dead man's switch. Robot will not accept any control input unless this button is pressed.
        D-Pad Up and Down - forward and backward linear motion.
        XYAB/1234 - speed multiplier modes. (1 - 25%, 2 - 50%, 3 - 75%, 4 - 100%)
    """

    def __init__(self):
        super().__init__('teleop')

        self._cmd_vel_pub = self.create_publisher(Float64, '/cmd_vel', 1)
        self._joy_sub = self.create_subscription(Joy, '/joy', self._joy_callback, 1)
        # self._stop_sub = self.create_subscription(String, 'realsense_dist', self._stop_callback, 10)

        timer_period = 1/2 # secs
        self._timer = self.create_timer(timer_period, self._cmd_callback)
        self._cmd_check = False

        self.max_vel = 500.0
        self.speed_multiplier = 0.25
        self.direction_multiplier = 0.0
        # self.stop_command = 1.0

    def _joy_callback(self, msg):
        """ Callback for joystick input. """

        if msg.buttons[7]: self._cmd_check = True
        else: self._cmd_check = False
        self.direction_multiplier = msg.axes[1]
        if msg.buttons[0]: self.speed_multiplier = 0.25
        elif msg.buttons[1]: self.speed_multiplier = 0.5
        elif msg.buttons[2]: self.speed_multiplier = 0.75
        elif msg.buttons[3]: self.speed_multiplier = 1.0
        # self.get_logger().info('Direction: %f, Speed: %f' % (self.direction_multiplier, self.speed_multiplier))

    # def _stop_callback(self, msg):
    #     """ Callback from RealSense threshold stop. """

    #     if bool(int(msg)): self.stop_command = 0.0
    #     else: self.stop_command = 1.0

    def _cmd_callback(self):
        """ Callback for command velocity output. """

        cmd_vel = Float64()
        cmd_vel.data = self.direction_multiplier * self.speed_multiplier * self.max_vel
        # cmd_vel.data = self.direction_multiplier * self.speed_multiplier * self.max_vel * self.stop_command
        if self._cmd_check:
            self._cmd_vel_pub.publish(cmd_vel)
            self.get_logger().info('Published velocity: %f' % cmd_vel.data)


def main():
    rclpy.init()
    node = JoystickController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
