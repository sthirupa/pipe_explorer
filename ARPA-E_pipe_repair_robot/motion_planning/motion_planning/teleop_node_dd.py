import time
import serial

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Float64MultiArray
from std_msgs.msg import String


class JoystickController(Node):
    """ Node to control the ARPA-E Pipe Repair robot with a joystick.
    
    Controls:
        RT - Dead man's switch. Robot will not accept any control input unless this button is pressed.
        D-Pad Up and Down - forward and backward linear motion.
        XYAB/1234 - speed multiplier modes. (1 - 25%, 2 - 50%, 3 - 75%, 4 - 100%)
    """

    def __init__(self):
        super().__init__('teleop_dd')

        self._r_cmd_vel_pub = self.create_publisher(Float64, '/repair_robot/r_cmd_vel', 1)
        self._l_cmd_vel_pub = self.create_publisher(Float64, '/repair_robot/l_cmd_vel', 1)
        self._cmd_vel_pub = self.create_publisher(Float64MultiArray, '/repair_robot/cmd_vel', 1)
        self._joy_sub = self.create_subscription(Joy, '/joy', self._joy_callback, 1)
        self._stop_sub = self.create_subscription(String, '/threshold_warning', self._stop_callback, 10)
        self._keyboard_sub = self.create_subscription(String, '/robot_key_cmd', self._keyboard_callback, 1)

        timer_period = 1/100 # secs
        self._timer = self.create_timer(timer_period, self._cmd_callback)
        self._cmd_check = False

        self.serial_port_open = False
        self.writer = serial.Serial()
        self._open_writer()

        self.max_vel = 0.75
        self.speed_multiplier = 0.25
        self.lr_direction_multiplier = 0.0
        self.fb_direction_multiplier = 0.0
        self.top_wheel_cmd = 'S'
        self.prev_cmd = 'S'
        self.stop_command = 1.0

    def _open_writer(self):

        _count = 0
        while not self.serial_port_open:
            try:
                self.writer.port = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_558383235353514112B2-if00'
                self.writer.baudrate = 115200
                self.writer.timeout = 1
                self.writer.open()
                self.serial_port_open = True
            except:
                print('Could not open serial port, retrying')
                _count += 1
                if _count > 5:
                    print('Could not open serial port, aborting')
                    break
                time.sleep(1)
                continue

    def _joy_callback(self, msg):
        """ Callback for joystick input. """

        if msg.buttons[7]: self._cmd_check = True
        else: self._cmd_check = False
        if msg.buttons[4]: self.top_wheel_cmd = 'D\n'
        elif msg.buttons[5]: self.top_wheel_cmd = 'U\n'
        else: self.top_wheel_cmd = 'S\n'
        self.fb_direction_multiplier = msg.axes[1] # front-back
        self.lr_direction_multiplier = msg.axes[0] # left-right
        if msg.buttons[0]: self.speed_multiplier = 0.25
        elif msg.buttons[1]: self.speed_multiplier = 0.5
        elif msg.buttons[2]: self.speed_multiplier = 0.75
        elif msg.buttons[3]: self.speed_multiplier = 1.0
        # self.get_logger().info('Direction: %f, Speed: %f' % (self.direction_multiplier, self.speed_multiplier))


    def _stop_callback(self, msg):
        """ Callback from RealSense threshold stop. """

        if bool(int(msg.data)): self.stop_command = 0.0
        else: self.stop_command = 1.0


    def _keyboard_callback(self, msg):
        """Callback from keyboard buttons system"""
        # s = 'Direction from input is ' + str(msg.data)
        # self.get_logger().info(s)

        # Removed first msg.data check to have continuous movement for MRSD SVD on 20230417
        # Commented out two lines below, changed elif "SlOW" to if "SLOW, changed else to elif != "0"
        # if msg.data == "0":
        #     self._cmd_check = False
        if msg.data == "SLOW":
            self.speed_multiplier = 0.25
        elif msg.data == "MEDIUM" or msg.data == "NORMAL":
            self.speed_multiplier = 0.50
        elif msg.data == "FAST":
            self.speed_multiplier = 0.75
        elif msg.data != "0":
            self._cmd_check = True

            if msg.data == "UP":
                self.fb_direction_multiplier = 1.0
                self.lr_direction_multiplier = 0.0
            elif msg.data == "DOWN":
                self.fb_direction_multiplier = -1.0
                self.lr_direction_multiplier = 0.0
            elif msg.data == "LEFT":
                self.fb_direction_multiplier = 0.0
                self.lr_direction_multiplier = 1.0
            elif msg.data == "RIGHT":
                self.fb_direction_multiplier = 0.0
                self.lr_direction_multiplier = -1.0

            # if msg.data == "SLOW":
            #     self.speed_multiplier = 0.25
            # elif msg.data == "MEDIUM" or msg.data == "NORMAL":
            #     self.speed_multiplier = 0.50
            # elif msg.data == "FAST":
            #     self.speed_multiplier = 0.75


    def _cmd_callback(self):
        """ Callback for command velocity output. """

        r_cmd_vel = Float64()
        l_cmd_vel = Float64()
        _cmd_vel = Float64MultiArray()

        if self.lr_direction_multiplier == 0.0:
            r_cmd_vel.data = -1.0 * self.fb_direction_multiplier * self.speed_multiplier * self.max_vel
            l_cmd_vel.data = self.fb_direction_multiplier * self.speed_multiplier * self.max_vel

            if self.fb_direction_multiplier > 0:
                r_cmd_vel.data = r_cmd_vel.data * self.stop_command
                l_cmd_vel.data = l_cmd_vel.data * self.stop_command

        elif self.fb_direction_multiplier == 0.0:
            r_cmd_vel.data = -1.0 * self.lr_direction_multiplier * self.speed_multiplier * self.max_vel * self.stop_command
            l_cmd_vel.data = -1.0 * self.lr_direction_multiplier * self.speed_multiplier * self.max_vel * self.stop_command
        else:
            r_cmd_vel.data = 0.0
            l_cmd_vel.data = 0.0
            if self.lr_direction_multiplier == 1.0:
                r_cmd_vel.data = self.fb_direction_multiplier * self.speed_multiplier * self.max_vel * self.stop_command
            elif self.lr_direction_multiplier == -1.0:
                l_cmd_vel.data = self.fb_direction_multiplier * self.speed_multiplier * self.max_vel * self.stop_command

        if self.serial_port_open and self.top_wheel_cmd != self.prev_cmd:
            print('top cmd, ', self.top_wheel_cmd)
            self.writer.write(self.top_wheel_cmd.encode())
            self.prev_cmd = self.top_wheel_cmd

        if self._cmd_check:
            self._r_cmd_vel_pub.publish(r_cmd_vel)
            self._l_cmd_vel_pub.publish(l_cmd_vel)
            speedreport = self.speed_multiplier * 100
            div = self.speed_multiplier * self.max_vel
            rightreport = -1 * r_cmd_vel.data / div
            leftreport = l_cmd_vel.data / div
            abs_velocity = float(abs(r_cmd_vel.data))

            if rightreport > 0:
                rightmsg = 'RIGHT-side Wheels driving FORWARD'
            else:
                rightmsg = 'RIGHT-side Wheels driving BACKWARD'

            if leftreport > 0:
                leftmsg = 'LEFT-side Wheels driving FORWARD'
            else:
                leftmsg = 'LEFT-side Wheels driving BACKWARD'

            self.get_logger().info('Speed setting in percentage of max: %f' % speedreport)
            self.get_logger().info(leftmsg)
            self.get_logger().info(rightmsg)
            self.get_logger().info('Published velocity (both sides): %f' % abs_velocity)
            # self.get_logger().info('Published velocity (right): %f' % r_cmd_vel.data)
            # self.get_logger().info('Published velocity (left): %f' % l_cmd_vel.data)
            self.get_logger().info(' ')

            _cmd_vel.data = [r_cmd_vel.data, l_cmd_vel.data]
            self._cmd_vel_pub.publish(_cmd_vel)
        else:
            r_cmd_vel.data = 0.0
            l_cmd_vel.data = 0.0
            self._r_cmd_vel_pub.publish(r_cmd_vel)
            self._l_cmd_vel_pub.publish(l_cmd_vel)

            _cmd_vel.data = [r_cmd_vel.data, l_cmd_vel.data]
            self._cmd_vel_pub.publish(_cmd_vel)


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
