import os
import cv2
import ipdb
import numpy as np

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer

from sensor_msgs.msg import Image


class BasicImageStitcher():
    """ Basic image stitcher class, stitches image by stacking """

    def __init__(self):
        pass

    def stitch(self, in_images, scale_percent = 50):

        width = int(in_images[0].shape[1] * scale_percent / 100)
        height = int(in_images[0].shape[0] * scale_percent / 100)
        out_size = (width, height)
        out_images = [cv2.resize(img, out_size) for img in in_images]
        stitched_image = cv2.hconcat(out_images)

        return stitched_image 

    # def stitch_thermal(self, in_images):

    #     scale_percent = 20
    #     width = int(in_images[0].shape[1] * scale_percent / 100)
    #     height = int(in_images[0].shape[0] * scale_percent / 100)
    #     out_size = (width, height)
    #     out_images = [cv2.resize(img, out_size) for img in in_images]


# TODO: stitcher with cone interpolation


class RGBImageStitcher(BasicImageStitcher):
    """ Node to stitch images from the 3 RGB cameras """

    def __init__(self):
        super().__init__()

        self.bridge = CvBridge()
        self.stitched_img = Image()

        # TODO find a good name for the images
        rclpy.init()
        self.node = rclpy.create_node('rgb_stitching')
        self._stitched_img_pub = self.node.create_publisher(Image, '/rgb/stitched_image', 1)

        # Listening to compressed rgb images instead of the raw images for stitching
        # self._input_img_1_sub = Subscriber(self.node, Image, '/rgb/camera_1/image_raw')
        # self._input_img_2_sub = Subscriber(self.node, Image, '/rgb/camera_2/image_raw')
        # self._input_img_3_sub = Subscriber(self.node, Image, '/rgb/camera_3/image_raw')

        self._input_img_1_sub = Subscriber(self.node, Image, '/rgb/camera_1/image_raw/compressed_rgb1')
        self._input_img_2_sub = Subscriber(self.node, Image, '/rgb/camera_2/image_raw/compressed_rgb2')
        self._input_img_3_sub = Subscriber(self.node, Image, '/rgb/camera_3/image_raw/compressed_rgb3')

        # Tried TimeSynchronizer, the 3 images dont share exact time stamps for sync
        # TODO is tuning required for slop (currently 0.2)?
        self.ts = ApproximateTimeSynchronizer([self._input_img_1_sub, self._input_img_2_sub, self._input_img_3_sub], 10, 0.1, allow_headerless=True)
        self.ts.registerCallback(self._input_img_callback)

        #Setting the frequency of publishing images (Sets the FPS we obtain on the base station)
        timer_period = 1 # secs
        self._timer = self.node.create_timer(timer_period, self._stitched_img_callback)

    def _input_img_callback(self, img1_msg, img2_msg, img3_msg):

        img1 = self.bridge.imgmsg_to_cv2(img1_msg)
        img2 = self.bridge.imgmsg_to_cv2(img2_msg)
        img3 = self.bridge.imgmsg_to_cv2(img3_msg)
        images = [img1, img2, img3]

        _img = self.stitch(images)
        self.stitched_img = self.bridge.cv2_to_imgmsg(_img)

    def _stitched_img_callback(self):

        self._stitched_img_pub.publish(self.stitched_img)

    def main(self):

        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            pass
        rclpy.shutdown()


class ThermalImageStitcher(Node, BasicImageStitcher):
    """ Node to stitch images from the 8 thermal cameras """

    def __init__(self):
        Node.__init__('thermal_stitching')
        pass


# TODO: maybe different classes for RGB and thermal image stitching, inheriting the same base class 

def main():

    stitcher = RGBImageStitcher()
    stitcher.main()


if __name__ == '__main__':
    main()

    ## TESTING WITHOUT ROS
    # stitcher = BasicImageStitcher()
    # test_img = cv2.imread('/home/arpa-e/pipe_crawler_ws/src/ARPA-E_pipe_repair/perception/InputImages/rocket.jpg')

    # img_rgb = [test_img, test_img, test_img]
    # img_thermal = [test_img*9]

    # stitched_rgb = stitcher.stitch_rgb(img_rgb)
    # cv2.imshow("stitched_rgb", stitched_rgb)
    # cv2.waitKey(5000)
