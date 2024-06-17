#!/usr/bin/env /usr/bin/python3
import numpy as np
import cv2


import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from jetson_utils import videoSource
class Stereo_Camera(Node):
    def __init__(self):
        super().__init__('stereo_camera')
        # CV Bridge
        self.cv_bridge = CvBridge()
        ## Subscriber ##

        ## Publisher ##
        self.left_cam_pub = self.create_publisher(Image, "/left_cam", 10)
        self.right_cam_pub = self.create_publisher(Image, "/right_cam", 10)

        self.right_cap = videoSource("csi://1")
        self.left_cap = videoSource("csi://0")

        timer_period = 1 / 60  # seconds
        self.timer = self.create_timer(timer_period, self.camera_callback)
    def camera_callback(self):
        left_bgr = self.left_cap.Capture(format='rgb8', timeout=1000)
        right_bgr = self.left_cap.Capture(format='rgb8', timeout=1000)

        if left_bgr is not None:
            left_bgr = np.array(left_bgr)
            left_image_msg = self.cv_bridge.cv2_to_imgmsg(left_bgr, "rgb8")
            left_image_msg.header.stamp = self.get_clock().now().to_msg()
            self.left_cam_pub.publish(left_image_msg)   # Publish left image
        if right_bgr is not None:
            right_bgr = np.array(right_bgr)
            right_image_msg = self.cv_bridge.cv2_to_imgmsg(right_bgr, "rgb8")
            right_image_msg.header.stamp = self.get_clock().now().to_msg()
            self.right_cam_pub.publish(right_image_msg)  # Publish right image
def main():
    rclpy.init()
    stereo_node = Stereo_Camera()
    rclpy.spin(stereo_node)
    stereo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()