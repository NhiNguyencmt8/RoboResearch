import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy


class FindObject(Node):

    def __init__(self):
        super().__init__('camera_subscriber')


        custom_qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy .BEST_EFFORT,
            history = QoSHistoryPolicy .KEEP_LAST,
            durability = QoSDurabilityPolicy .VOLATILE,
            depth = 1
        )

        # Creating a camera frame with object detection 
        self.image_publisher_ = self.create_publisher(CompressedImage, '/object_tracking', 10)
        self.back_sub = cv2.createBackgroundSubtractorMOG2(history=700, varThreshold=25, detectShadows=True)
        self.kernel = np.ones((20,20),np.uint8)
        self.bridge = CvBridge()

        self.point_publisher_ = self.create_publisher(Point, '/object_center', 5)

        # Creating a subcription from the camera_robot_launch
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            qos_profile=custom_qos_profile)
        
        self.subscription


    def image_callback(self, msg):
        # self.get_logger().info('I saw something on the camera!')
        frame = self.bridge.compressed_imgmsg_to_cv2(msg)
        frame_x = frame.shape[1]/2
        frame_y = frame.shape[0]/2
        self.get_logger().info(f"x: {frame_x: .2f}, y: {frame_y: .2f}")
        # Dealing with the image processing a bit

        red_lower1 = np.array([0, 70, 50], np.uint8)
        red_upper1 = np.array([10, 255, 255], np.uint8)
        red_lower2 = np.array([160, 70, 50], np.uint8)
        red_upper2 = np.array([180, 255, 255], np.uint8)

        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(hsv_image, red_lower1, red_upper1)
        mask2 = cv2.inRange(hsv_image, red_lower2, red_upper2)

        # Combine both masks to cover the full red hue spectrum
        red_mask = mask1 | mask2
        
        kernel = np.ones((5, 5), "uint8") 
        
        # For red color 
        red_mask = cv2.dilate(red_mask, kernel) 
        # Creating contour to track red color 
        contours, hierarchy = cv2.findContours(red_mask, 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE) 
        
        if contours: 
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour) 
            frame = cv2.rectangle(frame, (x, y),  
                                    (x + w, y + h),  
                                    (0, 0, 255), 2) 
            xc = x + int(w/2)
            yc = y + int(h/2)
            cv2.circle(frame, (xc, yc), 4, (0, 255, 0), -1)
            text = "x: " + str(xc) + ", y: " + str(yc)
            cv2.putText(frame, text, (xc - 10, yc - 10), cv2.FONT_HERSHEY_PLAIN, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, "Aiyo it's Red", (x, y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, 
                        (0, 0, 255))
        else:
            xc = -1.0
            yc = -1.0 
        self.get_logger().info('Red object at ' + str(xc) + ',' + str(yc))   
        self.publish_point(float(xc),float(yc))   
    
        # Publishing the code to the topic
        self.publish_image(frame)

    def publish_point(self, xc, yc):
        center_point = Point()
        center_point.x = xc
        center_point.y = yc
        self.point_publisher_.publish(center_point)

    def publish_image(self, frame):
        # Convert from cv2 type to image message type so we can view it from the image_view=<our_topic>
        image_message = self.bridge.cv2_to_compressed_imgmsg(frame)
        self.image_publisher_.publish(image_message)

def main(args=None):
    rclpy.init(args=args)

    object_tracking = FindObject()
    rclpy.spin(object_tracking)

    object_tracking.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()