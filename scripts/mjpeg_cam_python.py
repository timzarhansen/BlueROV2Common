#!/usr/bin/env python3

import threading

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class CameraNode(Node):

    def __init__(self):
        super().__init__('mjpeg_camera')
        self.bridge = CvBridge()
        self.device = cv2.VideoCapture(0, apiPreference=cv2.CAP_V4L2)
        fourcc = 'MJPG'
        self.device.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self.device.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        self.device.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
        self.device.set(cv2.CAP_PROP_FPS, 15.0)

        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        self.image_pub = self.create_publisher(CompressedImage,
                                               'image_raw/compressed', qos)

        self.grab_thread = threading.Thread(target=self.grab_worker)
        self.grab_thread.start()

    def grab_worker(self):
        import time
        time.sleep(1.0)
        while rclpy.ok():
            success, frame = self.device.read()
            if success:
                # print("test1")
                # continue
                msg = self.bridge.cv2_to_compressed_imgmsg(frame)
                self.image_pub.publish(msg)
                # print("test2")
                continue
                print("test3")
                msg = CompressedImage()
                msg.data = frame.tostring()
                msg.format = 'jpeg'
                self.image_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
