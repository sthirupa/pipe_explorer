import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np
import cv2

# Image resolution of Thermal Cameras
THERMAL_RAW_IMG_W, THERMAL_RAW_IMG_H = [240, 640]

#Compressing by a scale of THERMAL_FX, THERMAL_FY (used in compress_thermal())
THERMAL_FX, THERMAL_FY = (2, 2)

class CompressThermal(Node):

    def __init__(self):
        super().__init__('compress_thermal')

        #Subscribers for images published by the thermal cameras
        self.thermal_sub_ = self.create_subscription(Image, '/thermal/image_raw', self.compress_thermal, 10)
        
        #Publishers for compressed thermal image
        self.thermal_pub_ = self.create_publisher(Image, '/thermal/image_raw/compressed_thermal', 10)
        
        #Setting the frequency of publishing images (Sets the FPS we obtain on the base station)
        timer_period = 1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        #Initializing
        self.br = CvBridge()
        self.compressed_thermal_ = np.zeros((int(THERMAL_RAW_IMG_W/THERMAL_FX), int(THERMAL_RAW_IMG_H/THERMAL_FY), 3))
    
    # Compressing using a simple cv2.resize
    # Doc: https://docs.opencv.org/3.4/da/d54/group__imgproc__transform.html#ga47a974309e9102f5f08231edc7e7529d
    
    def compress_thermal(self, data):

        self.compressed_thermal_ = cv2.resize(self.br.imgmsg_to_cv2(data), None, fx=(1/THERMAL_FX), fy=(1/THERMAL_FY), interpolation=cv2.INTER_AREA)

    
    def timer_callback(self):
        
        self.thermal_pub_.publish(self.br.cv2_to_imgmsg(self.compressed_thermal_))


def main():
    rclpy.init()
    node = CompressThermal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()