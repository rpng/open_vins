#!/usr/bin/env python
# simply tool to take two left/right images and to stack them ontop of each other
# to make it easier to visualise the tracks from OpenVINS

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class StackImage(Node):
    def __init__(self):
        super().__init__('stack_image')
        self.bridge = CvBridge()
        
        topic_in = "/ov_msckf/trackhist"
        topic_out = topic_in + "_rotated"
        print(" input: " + topic_in)
        print("output: " + topic_out)

        self.subscription = self.create_subscription(
            Image,
            topic_in,  # Input topic
            self.image_callback,
            10)
        
        self.publisher = self.create_publisher(
            Image,
            topic_out,  # Output topic
            10)

        self.get_logger().info('StackImage node has started.')
        
        self.counter = 0

    def image_callback(self, msg):
        if (self.counter % 25 == 0):
            str_out = str(self.counter) + " " + str(msg.header.stamp.sec) + " " + str(msg.header.stamp.nanosec)
            self.get_logger().info(str_out)
        self.counter = self.counter + 1
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rotated = cv2.rotate(cv_image, cv2.ROTATE_90_CLOCKWISE)
            
            # Get image dimensions
            height, width = cv_image.shape[:2]

            # Split image into left and right halves
            left_half = cv_image[:, :width // 2]
            right_half = cv_image[:, width // 2:]

            # Resize left_half to match the width of right_half if needed
            if left_half.shape[1] != right_half.shape[1]:
                left_half = cv2.resize(left_half, (right_half.shape[1], height))

            # Concatenate vertically: left half on top, right half on bottom
            result = np.concatenate((left_half, right_half), axis=0)            
            
            rotated_msg = self.bridge.cv2_to_imgmsg(result, encoding='bgr8')
            rotated_msg.header = msg.header  # Preserve timestamp and frame_id
            self.publisher.publish(rotated_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = StackImage()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
