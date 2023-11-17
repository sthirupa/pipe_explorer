import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Float64, Float64MultiArray
import numpy as np

class ImuExtractor(Node):
    def __init__(self):
        super().__init__('imu_extractor')

        # Create publishers for individual components
        self.orientation_publisher = self.create_publisher(Float64MultiArray, '/imu_orientation', 10)
        self.angular_velocity_publisher = self.create_publisher(Float64MultiArray, '/imu_angular_velocity', 10)
        self.linear_acceleration_publisher = self.create_publisher(Float64MultiArray, '/imu_linear_acceleration', 10)

        # Create subscriber to receive the IMU message
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def imu_callback(self, msg):
        # Extract and publish orientation
        orientation_msg = Float64MultiArray()
        orientation_msg.data = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.orientation_publisher.publish(orientation_msg)

        # Extract and publish angular velocity as Float64MultiArray
        angular_velocity_msg = Float64MultiArray()
        angular_velocity_msg.data = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        self.angular_velocity_publisher.publish(angular_velocity_msg)

        # Extract and publish linear acceleration as Float64MultiArray
        linear_acceleration_msg = Float64MultiArray()
        linear_acceleration_msg.data = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        self.linear_acceleration_publisher.publish(linear_acceleration_msg)

def main(args=None):
    rclpy.init(args=args)
    imu_extractor = ImuExtractor()
    rclpy.spin(imu_extractor)
    imu_extractor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()