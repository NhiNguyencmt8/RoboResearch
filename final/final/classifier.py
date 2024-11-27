#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int8
import cv2
import numpy as np
from cv_bridge import CvBridge
import pickle
import random

class Classifier:

    def __init__(self):
        rospy.init_node('classifier', anonymous=True)

        self.svm_classifier = pickle.load(open('svm_model.pickle', 'rb'))
        self.bridge = CvBridge()

        rospy.Subscriber('/image_raw/compressed', CompressedImage, self.image_callback, queue_size=1)

        self.prediction_publisher = rospy.Publisher('/prediction', Int8, queue_size=10)
        self.classified_image_publisher = rospy.Publisher('/classifier_image', CompressedImage, queue_size=10)

    def preprocess_image(self, image):
        alpha = random.uniform(0.8, 1.2)
        beta = random.randint(-30, 30)
        adjusted = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
        gray = cv2.cvtColor(adjusted, cv2.COLOR_BGR2GRAY)
        resized = cv2.resize(gray, (150, 150))
        return resized

    def extract_hog_features(self, image):
        hog = cv2.HOGDescriptor((64, 64), (16, 16), (8, 8), (8, 8), 9)
        return hog.compute(image).flatten()

    def predict(self, image):
        preprocessed_image = self.preprocess_image(image)
        features = self.extract_hog_features(preprocessed_image)
        prediction = self.svm_classifier.predict([features])[0]
        return prediction

    def image_callback(self, msg):
        try:
            image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            predicted_label = self.predict(image)
            prediction_msg = Int8()
            prediction_msg.data = int(predicted_label)
            self.prediction_publisher.publish(prediction_msg)

            classified_image_msg = self.bridge.cv2_to_compressed_imgmsg(image)
            self.classified_image_publisher.publish(classified_image_msg)

        except Exception as e:
            rospy.logerr(f"Error in image_callback: {e}")

if __name__ == '__main__':
    try:
        classifier_node = Classifier()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
