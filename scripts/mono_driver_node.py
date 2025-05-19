#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

from message_filters import Subscriber, ApproximateTimeSynchronizer

class CompressedRGBDNode(Node):
    def __init__(self):
        super().__init__('compressed_rgbd_node')

        self.bridge = CvBridge()

        # 建立 publisher：分別發送 RGB 與 Depth 圖像
        self.rgb_pub = self.create_publisher(Image, '/mono_py_driver/rgb_image', 10)
        self.depth_pub = self.create_publisher(Image, '/mono_py_driver/depth_image', 10)

        # 建立設定檔 publisher
        self.config_pub = self.create_publisher(String, '/mono_py_driver/experiment_settings', 10)
        self.timer = self.create_timer(2.0, self.publish_config)
        self.config_sent = False

        # 使用 message_filters 同步 RGB 與 Depth
        self.rgb_sub = Subscriber(self, CompressedImage, '/camera/image/compressed')
        self.depth_sub = Subscriber(self, CompressedImage, '/camera/depth/compressed')

        self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.synced_callback)

        self.get_logger().info('Node started. Waiting for synchronized RGB and depth images...')

    def publish_config(self):
        if not self.config_sent:
            msg = String()
            msg.data = "PROS"
            self.config_pub.publish(msg)
            self.get_logger().info("Published experiment setting: PROS")
            self.config_sent = True

    def synced_callback(self, rgb_msg: CompressedImage, depth_msg: CompressedImage):
        try:
            # 解碼 RGB
            rgb_arr = np.frombuffer(rgb_msg.data, np.uint8)
            rgb_image = cv2.imdecode(rgb_arr, cv2.IMREAD_COLOR)
            rgb_image_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding="bgr8")
            rgb_image_msg.header.stamp = rgb_msg.header.stamp
            self.rgb_pub.publish(rgb_image_msg)

            # 解碼 Depth
            depth_arr = np.frombuffer(depth_msg.data, np.uint8)
            depth_image = cv2.imdecode(depth_arr, cv2.IMREAD_UNCHANGED)
            depth_image_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
            depth_image_msg.header.stamp = depth_msg.header.stamp
            self.depth_pub.publish(depth_image_msg)

            self.get_logger().info("Published synchronized RGB and depth images")

        except Exception as e:
            self.get_logger().error(f"Failed to process images: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CompressedRGBDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
