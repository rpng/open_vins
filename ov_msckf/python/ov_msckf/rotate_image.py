#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageRotator90(Node):
    def __init__(self):
        super().__init__('image_rotator_90')
        self.bridge = CvBridge()
        
        topic_in = "/ov_msckf/trackhist"
        topic_out = topic_in + "_rotated"

        self.subscription = self.create_subscription(
            Image,
            topic_in,  # Input topic
            self.image_callback,
            10)
        
        self.publisher = self.create_publisher(
            Image,
            topic_out,  # Output topic
            10)

        self.get_logger().info('ImageRotator90 node has started.')
        
        self.counter = 0

    def image_callback(self, msg):
        if (self.counter % 25 == 0):
            str_out = str(self.counter) + " " + str(msg.header.stamp.sec) + " " + str(msg.header.stamp.nanosec)
            self.get_logger().info(str_out)
        self.counter = self.counter + 1
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rotated = cv2.rotate(cv_image, cv2.ROTATE_90_CLOCKWISE)
            rotated_msg = self.bridge.cv2_to_imgmsg(rotated, encoding='bgr8')
            rotated_msg.header = msg.header  # Preserve timestamp and frame_id
            self.publisher.publish(rotated_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageRotator90()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
