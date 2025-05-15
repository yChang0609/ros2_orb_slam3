#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class CompressedToRawNode(Node):
    def __init__(self):
        super().__init__('compressed_to_raw_node')

        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, '/mono_py_driver/img_msg', 10)
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self.listener_callback,
            10)

        # 發送 exp setting（模擬）
        self.config_publisher_ = self.create_publisher(String, '/mono_py_driver/experiment_settings', 10)
        self.timer = self.create_timer(2.0, self.publish_config)

        self.config_sent = False
        self.get_logger().info('Node started. Waiting for compressed image...')

    def publish_config(self):
        if not self.config_sent:
            msg = String()
            msg.data = "PROS"  # 對應 ORB-SLAM3 設定檔名稱（會 append `.yaml`）
            self.config_publisher_.publish(msg)
            self.get_logger().info("Published experiment setting: PROS")
            self.config_sent = True

    def listener_callback(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        try:
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            image_msg.header.stamp = msg.header.stamp  # 保留時間戳
            self.publisher_.publish(image_msg)
            self.get_logger().info("Published raw image")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CompressedToRawNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
