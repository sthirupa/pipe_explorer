import os
import cv2
import time
from struct import *
from itertools import chain

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image

VIDEO_PATH = '/dev/video'


class ThermalCameraDriver(Node):

    def __init__(self):
        super().__init__('flir_thermal_camera')

        (self.available, self.working, self.target) = self.list_ports()
        self.vid = cv2.VideoCapture(self.target[0])
        self.bridge = CvBridge()

        self._theral_img_pub = self.create_publisher(Image, '/thermal/image_raw', 1)

        timer_period = 1/30 # secs
        self._timer = self.create_timer(timer_period, self._thermal_img_callback)
        
    def list_ports(self):

        active_port = True
        dev_port = 0
        working_ports = []
        available_ports = []
        target_port = []

        while active_port:
            if not os.path.exists(VIDEO_PATH + str(dev_port)): active_port = False
            camera = cv2.VideoCapture(dev_port)
            if not camera.isOpened():
                print("Port %s is not working." %dev_port)
            else:
                is_reading, img = camera.read()
                w = camera.get(3)
                h = camera.get(4)
                if is_reading:
                    print("Port %s is working and reads images (%s x %s)" %(dev_port,h,w))
                    available_ports.append(dev_port)
                    working_ports.append(dev_port)
                else:
                    print("Port %s for camera ( %s x %s) is present but does not reads." %(dev_port,h,w))
                    available_ports.append(dev_port)
                if(h == 240 and w == 640):
                    target_port.append(dev_port)
            dev_port +=1

        return available_ports,working_ports,target_port

    def to_matrix(self, l, n):
        return [l[i:i+n] for i in range(0, len(l), n)]

    def get_telemetry(self, frame):

        n_cameras = 8
        format_str = "HHHHLLHH" #BHHHHLHH
        array = bytearray(list(chain.from_iterable(frame[0])))
        scaled_format_str = "=" + (n_cameras*format_str)
        numbers = unpack(scaled_format_str, array[0:calcsize(scaled_format_str)])
        cameras = self.to_matrix(numbers, len(format_str))
        formatted = []

        for camera in cameras:
            temperature = (camera[1] / 100.0) - 273.15
            valid = "valid" if (camera[0] == 1) else "invalid"
            uptime = camera[5] / 1000.0
            auxTemp = (camera[6] / 100.0) - 273.15
            fpaTemp = (camera[7] / 100.0) - 273.15
            formatted += [[valid, temperature, auxTemp, fpaTemp, uptime]]
        
        return formatted

    def _thermal_img_callback(self):
        
        t_last_frame = time.time()
        avg_list = []
        thermal_img = Image()

        ret, frame = self.vid.read()
        #Frame needs to be flipped on Linux!
        frame = cv2.flip(frame, 0)
        print("TELEMETRY")
        telemetry = self.get_telemetry(frame)
        output = ['', '', '']
        count = 1
        for camera in telemetry:
            output[0] += 'C{}\t'.format(count)
            output[1] += '{} \t'.format(camera[0])
            output[2] += '{:2.2f}C \t'.format(camera[1])
            count+=1

        print(output[0])
        print(output[1])
        print(output[2])

        thermal_img = self.bridge.cv2_to_imgmsg(frame)
        self._theral_img_pub.publish(thermal_img)
        
        # cv2.imshow('frame', frame)

        d_t = time.time() - t_last_frame
        t_last_frame = time.time()
        avg_list.append(1/d_t)

        if(len(avg_list) > 100):
            avg_list = avg_list[1:]

        print('FPS:{:2.2f} \t Uptime:{:2.0f}s'.format(sum(avg_list) / len(avg_list), telemetry[0][4]))
        print()


def main():
    rclpy.init()
    node = ThermalCameraDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.vid.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()