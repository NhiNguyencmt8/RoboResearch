import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np


class DisplayNode(Node):

    def __init__(self):
        super().__init__('display_node')
        self.bridge = CvBridge()

        # Creating a subcription from the camera_robot_launch
        self.subscription = self.create_subscription(
            CompressedImage,
            '/object_tracking',
            self.display_callback,
            10)
        self.subscription
        self.back_sub = cv2.createBackgroundSubtractorMOG2(history=700, varThreshold=25, detectShadows=True)
        self.kernel = np.ones((20, 20), np.uint8)
        cv2.namedWindow("Display", cv2.WINDOW_AUTOSIZE)

    def display_callback(self, msg):
        self.get_logger().info('Heard from object_tracking')   
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.namedWindow("Display", cv2.WINDOW_AUTOSIZE) 
        cv2.imshow('Display', frame) 
        cv2.waitKey(1)
        
def main(args=None):
    rclpy.init(args=args)

    display_node = DisplayNode()
    rclpy.spin(display_node)

    display_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
