#!/usr/bin/env python3
import cv2
import numpy as np
import csv
import math
import random
from sklearn.metrics import classification_report
from sklearn.preprocessing import StandardScaler

# Load training and testing data
imageDirectory = './2024F_imgs/'
imageType = '.png'

with open(imageDirectory + 'labels.txt', 'r') as f:
    reader = csv.reader(f)
    lines = list(reader)

# Randomly choose train and test data (50/50 split)
random.shuffle(lines)
train_lines = lines[:math.floor(len(lines) / 2)][:]  # Training data
test_lines = lines[math.floor(len(lines) / 2):][:]   # Testing data

def extract_HOG(image):
    """
    Extract HOG features from an image.
    """
    # Define HOGDescriptor with required parameters
    winSize = (64, 64)  # Detection window size
    blockSize = (16, 16)  # Block size
    blockStride = (8, 8)  # Block stride
    cellSize = (8, 8)  # Cell size
    nbins = 9  # Number of bins

    # Create the HOG descriptor object
    hog = cv2.HOGDescriptor(winSize, blockSize, blockStride, cellSize, nbins)

    # Compute HOG features for the input image
    return hog.compute(image).flatten()

# Preprocess training data
train_data = []
train_labels = []
for i in range(len(train_lines)):
    original_img = cv2.imread(imageDirectory + train_lines[i][0] + imageType)
    grayscale = cv2.cvtColor(original_img, cv2.COLOR_BGR2GRAY)
    resized_img = cv2.resize(grayscale, (64, 64))  # Higher resolution
    features = extract_HOG(resized_img)
    train_data.append(features)
    train_labels.append(int(train_lines[i][1]))

train_data = np.array(train_data, dtype=np.float32)
train_labels = np.array(train_labels, dtype=np.int32)

# Preprocess testing data
test_data = []
test_labels = []
for i in range(len(test_lines)):
    original_img = cv2.imread(imageDirectory + test_lines[i][0] + imageType)
    grayscale = cv2.cvtColor(original_img, cv2.COLOR_BGR2GRAY)
    resized_img = cv2.resize(grayscale, (64, 64))  # Higher resolution
    features = extract_HOG(resized_img)
    test_data.append(features)
    test_labels.append(int(test_lines[i][1]))

test_data = np.array(test_data, dtype=np.float32)
test_labels = np.array(test_labels, dtype=np.int32)

# Train SVM classifier
svm = cv2.ml.SVM_create()
svm.setType(cv2.ml.SVM_C_SVC)
svm.setKernel(cv2.ml.SVM_RBF)  # Use RBF kernel
svm.setC(100.0)  # Adjust regularization strength
svm.setGamma(0.01)  # Experiment with gamma

svm.train(train_data, cv2.ml.ROW_SAMPLE, train_labels)

# Test SVM classifier
correct = 0
confusion_matrix = np.zeros((6, 6))  # Assuming 6 classes

for i in range(len(test_lines)):
    test_sample = test_data[i].reshape(1, -1)  # Reshape for prediction
    test_label = test_labels[i]

    _, result = svm.predict(test_sample)
    predicted_label = int(result[0, 0])

    if test_label == predicted_label:
        correct += 1
        confusion_matrix[test_label][predicted_label] += 1
    else:
        confusion_matrix[test_label][predicted_label] += 1
        print(f"{test_lines[i][0]} Wrong, {test_label} classified as {predicted_label}")

accuracy = correct / len(test_lines)
print("\n\nTotal accuracy: {:.2f}%".format(accuracy * 100))
print("Confusion Matrix:")
print(confusion_matrix)
