#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
import cv2
import numpy as np
from cv_bridge import CvBridge
import pickle
import random

class Classifier(Node):

    def __init__(self):
        super().__init__('classifier')
        
        # Load SVM model
        self.svm_classifier = pickle.load(open('/home/notsean/ros_ws/src/final/final/svm_model.pickle', 'rb'))
        
        # Load SelectKBest model
        self.feature_selector = pickle.load(open('/home/notsean/ros_ws/src/final/final/feature_selector.pickle', 'rb'))
        
        # Initialize CvBridge
        self.bridge = CvBridge()

        # Create subscribers
        self.image_subscriber = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        # Create publishers
        self.prediction_publisher = self.create_publisher(Int8, '/prediction', 10)

    def rgb_to_cmyk(self, image):
        bgr = image.astype(np.float32) / 255.0
        r, g, b = bgr[..., 2], bgr[..., 1], bgr[..., 0]
        k = 1 - np.max([r, g, b], axis=0)
        c = (1 - r - k) / (1 - k + 1e-8)
        m = (1 - g - k) / (1 - k + 1e-8)
        y = (1 - b - k) / (1 - k + 1e-8)
        cmyk = np.stack((c, m, y, k), axis=-1) * 255
        return cmyk.astype(np.uint8)

    def adjust_brightness_contrast(self, image):
        alpha = random.uniform(0.8, 1.2)
        beta = random.randint(-30, 30) 
        return cv2.convertScaleAbs(image, alpha=alpha, beta=beta)

    def extract_HOG(self, image):
        winSize = (64, 64) 
        blockSize = (16, 16)
        blockStride = (8, 8)
        cellSize = (8, 8)
        nbins = 9
        hog = cv2.HOGDescriptor(winSize, blockSize, blockStride, cellSize, nbins)
        # Compute HOG features
        return hog.compute(image).flatten()

    def extract_features_cymk(self, image):
        cmyk_img = self.rgb_to_cmyk(image)
        grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        c_features = self.extract_HOG(cv2.resize(cmyk_img[..., 0], (64, 64)))
        m_features = self.extract_HOG(cv2.resize(cmyk_img[..., 1], (64, 64)))
        y_features = self.extract_HOG(cv2.resize(cmyk_img[..., 2], (64, 64)))
        k_features = self.extract_HOG(cv2.resize(cmyk_img[..., 3], (64, 64)))

        gray_features = self.extract_HOG(cv2.resize(grayscale, (64, 64)))

        return np.concatenate((c_features, m_features, y_features, k_features, gray_features))

    def preprocess_image(self, image):
        augmented_image = self.adjust_brightness_contrast(image)
        features = self.extract_features_cymk(augmented_image)
        # Apply feature selection
        selected_features = self.feature_selector.transform([features])[0]
        return selected_features

    def predict(self, image):
        preprocessed_image = self.preprocess_image(image)
        prediction = self.svm_classifier.predict([preprocessed_image])[0]
        return prediction

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Perform prediction
            predicted_label = self.predict(image)

            # Publish the prediction
            prediction_msg = Int8()
            prediction_msg.data = int(predicted_label)
            self.prediction_publisher.publish(prediction_msg)

        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    classifier_node = Classifier()

    # Spin to keep the node running
    rclpy.spin(classifier_node)

    # Cleanup after spin ends
    classifier_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
