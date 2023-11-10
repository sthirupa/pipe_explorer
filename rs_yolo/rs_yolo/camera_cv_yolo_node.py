import time
import serial

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Joy, Image
from std_msgs.msg import Float64, Float64MultiArray, String
from .camera_cv_yolo_main import yolo_main


class CVYoloNode(Node):
    def __init__(self):
        super().__init__('camera_cv_yolo_node')
        
        self._warning_pub = self.create_publisher(String, '/threshold_warning', 1)
        self._yolo_sub = self.create_subscription(Image, '/color/image_raw', self.yolo_callback, 10)
        self.cv_bridge = CvBridge()
        self._coordinates_pub = self.create_publisher(Float64MultiArray, '/box_coordinates', 10)

    def yolo_callback(self, msg):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        warning_msg = String()
        warning_msg.data = '0'
        try:
            warning_msg.data, coordinates = yolo_main(cv_image)
        except Exception as e:
            # self.get_logger().error("Error converting Image message: %s" % str(e))
            print("Error converting Image msg: %s" % str(e))

        # Maybe remove the warning_msg?
        self._warning_pub.publish(warning_msg)
        if coordinates:
            coordinates_msg = Float64MultiArray()
            coordinates_msg.data = coordinates
            self._coordinates_pub.publish(coordinates_msg)


def main():
    rclpy.init()
    node = CVYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()