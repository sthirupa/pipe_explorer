import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np
import cv2

# Image resolution of RGB Cameras
RGB_RAW_IMG_W, RGB_RAW_IMG_H = [920, 1224]

#Compressing by a scale of RGB_FX, RGB_FY (used in compress_rgb_*())
RGB_FX, RGB_FY = (4, 4)

class CompressRGB(Node): 

    def __init__(self):
        super().__init__('compress_rgb')

        #Subscribers for images published by the three cameras
        self.rgb_1_sub_ = self.create_subscription(Image, '/rgb/camera_1/image_raw', self.compress_rgb_1, 10)
        self.rgb_2_sub_ = self.create_subscription(Image, '/rgb/camera_2/image_raw', self.compress_rgb_2, 10)
        self.rgb_3_sub_ = self.create_subscription(Image, '/rgb/camera_3/image_raw', self.compress_rgb_3, 10)
        
        # Publishers for images compressed of the respective three cameras
        self.rgb_1_pub_ = self.create_publisher(Image, '/rgb/camera_1/image_raw/compressed_rgb1', 10)
        self.rgb_2_pub_ = self.create_publisher(Image, '/rgb/camera_2/image_raw/compressed_rgb2', 10)
        self.rgb_3_pub_ = self.create_publisher(Image, '/rgb/camera_3/image_raw/compressed_rgb3', 10)
        
        #Setting the frequency of publishing images (Sets the FPS we obtain on the base station)
        timer_period = 1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        #Initializing        
        self.br = CvBridge()
        self.compressed_rgb_image_1_ = np.zeros((int(RGB_RAW_IMG_W/RGB_FY), int(RGB_RAW_IMG_H/RGB_FY), 3))
        self.compressed_rgb_image_2_ = np.zeros((int(RGB_RAW_IMG_W/RGB_FX), int(RGB_RAW_IMG_H/RGB_FY), 3))
        self.compressed_rgb_image_3_ = np.zeros((int(RGB_RAW_IMG_W/RGB_FX), int(RGB_RAW_IMG_H/RGB_FY), 3))
    
    # Compressing using a simple cv2.resize
    # Doc: https://docs.opencv.org/3.4/da/d54/group__imgproc__transform.html#ga47a974309e9102f5f08231edc7e7529d
        
    def compress_rgb_1(self, data):

        self.compressed_rgb_image_1_ = cv2.resize(self.br.imgmsg_to_cv2(data), None, fx=(1/RGB_FX), fy=(1/RGB_FY), interpolation=cv2.INTER_AREA)

    def compress_rgb_2(self, data):

        self.compressed_rgb_image_2_ = cv2.resize(self.br.imgmsg_to_cv2(data), None, fx=(1/RGB_FX), fy=(1/RGB_FY), interpolation=cv2.INTER_AREA)

    def compress_rgb_3(self, data):

        self.compressed_rgb_image_3_ = cv2.resize(self.br.imgmsg_to_cv2(data), None, fx=(1/RGB_FX), fy=(1/RGB_FY), interpolation=cv2.INTER_AREA)

    def timer_callback(self):
        
        self.rgb_1_pub_.publish(self.br.cv2_to_imgmsg(self.compressed_rgb_image_1_))
        self.rgb_2_pub_.publish(self.br.cv2_to_imgmsg(self.compressed_rgb_image_2_))
        self.rgb_3_pub_.publish(self.br.cv2_to_imgmsg(self.compressed_rgb_image_3_))


def main():
    rclpy.init()
    node = CompressRGB()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()