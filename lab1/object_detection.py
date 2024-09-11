import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np



class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        # Creating a camera frame with object detection 
        self.image_publisher_ = self.create_publisher(Image, '/object_tracking', 10)
        self.back_sub = cv2.createBackgroundSubtractorMOG2(history=700, varThreshold=25, detectShadows=True)
        self.kernel = np.ones((20,20),np.uint8)
        self.bridge = CvBridge()

        # Creating a subcription from the turtlebot3 camera
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10)
        self.subscription


    def image_callback(self, msg):
        self.get_logger().info('I saw something on the camera!')
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Dealing with the image processing a bit
        hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
        red_lower = np.array([136, 87, 111], np.uint8) 
        red_upper = np.array([180, 255, 255], np.uint8) 
        red_mask = cv2.inRange(hsvFrame, red_lower, red_upper) 
        kernel = np.ones((5, 5), "uint8") 
      
        # For red color 
        red_mask = cv2.dilate(red_mask, kernel) 
        # Creating contour to track red color 
        contours, hierarchy = cv2.findContours(red_mask, 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE) 
        
        for pic, contour in enumerate(contours): 
            area = cv2.contourArea(contour) 
            if(area > 300): 
                x, y, w, h = cv2.boundingRect(contour) 
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
                self.get_logger().info('Red object at ' + str(xc) + ',' + str(yc))      
    
        # Publishing the code to the topic
        self.publish_image(frame)

    def publish_image(self, frame):
        # Convert from cv2 type to image message type so we can view it from the image_view=<our_topic>
        image_message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_publisher_.publish(image_message)

def main(args=None):
    rclpy.init(args=args)

    object_tracking = CameraNode()
    rclpy.spin(object_tracking)

    object_tracking.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
