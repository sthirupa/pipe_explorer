import rclpy
import sensor_msgs
from sensor_msgs.msg import Image
import sensor_msgs
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np
import cv2

# Image resolution of Realsense
RGB_RAW_IMG_W, RGB_RAW_IMG_H  = [720, 1280]
DEPTH_RAW_IMG_W, DEPTH_RAW_IMG_H  = [480, 640]

#Compressing by a scale of RS_FX, RS_FY (used in compress_rgb_image() and compress_depth_image())
RS_FX, RS_FY = (4, 4)

class CompressRealsense(Node):

    def __init__(self):
        super().__init__('compress_realsense')
        
        #Subscribers for rgb image and depth image
        self.rgb_sub_ = self.create_subscription(Image, '/camera/color/image_raw', self.compress_rgb_image, 10)
        self.depth_sub_ = self.create_subscription(Image, '/camera/depth/image_rect_raw', self.compress_depth_image, 10)

        #Publishers for compressed rgb image and compressed depth image
        self.rgb_pub_ = self.create_publisher(Image, '/camera/color/image_raw/compressed_realsense_rgb', 10)
        self.depth_pub_ = self.create_publisher(Image, '/camera/depth/image_rect_raw/compressed_realsense_depth', 10)
        
        #Setting the frequency of publishing images (Sets the FPS we obtain on the base station)
        timer_period = 1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        #Initializing
        self.br = CvBridge()
        self.compressed_rgb_image_ = np.zeros((int(RGB_RAW_IMG_W/RS_FX), int(RGB_RAW_IMG_H/RS_FY),3), dtype='uint8')
        self.compressed_depth_image_ = np.empty((int(DEPTH_RAW_IMG_W/RS_FX), int(DEPTH_RAW_IMG_H/RS_FY)), dtype='uint16')
    
    # Compressing using a simple cv2.resize
    # Doc: https://docs.opencv.org/3.4/da/d54/group__imgproc__transform.html#ga47a974309e9102f5f08231edc7e7529d
    
    def compress_rgb_image(self, data):

        self.compressed_rgb_image_ = cv2.resize(self.br.imgmsg_to_cv2(data, 'rgb8'), None, fx=(1/RS_FX), fy=(1/RS_FY), interpolation=cv2.INTER_AREA)

    def compress_depth_image(self, data):
        
        self.compressed_depth_image_ = cv2.resize(self.br.imgmsg_to_cv2(data, '16UC1'), None, fx=(1/RS_FX), fy=(1/RS_FY), interpolation=cv2.INTER_AREA)


    def timer_callback(self):
        
        self.rgb_pub_.publish(self.br.cv2_to_imgmsg(self.compressed_rgb_image_, 'rgb8'))
        self.depth_pub_.publish(self.br.cv2_to_imgmsg(self.compressed_depth_image_, '16UC1'))

def main():
    rclpy.init()
    node = CompressRealsense()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
