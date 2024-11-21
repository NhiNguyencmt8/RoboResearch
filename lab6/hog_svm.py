#!/usr/bin/env python3
import cv2
import numpy as np
import csv
import math
import random
from sklearn.preprocessing import MinMaxScaler
from sklearn.feature_selection import SelectKBest, f_classif
from sklearn.model_selection import cross_val_score
from sklearn.svm import SVC
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

def rgb_to_cmyk(image):
    bgr = image.astype(np.float32) / 255.0
    r, g, b = bgr[..., 2], bgr[..., 1], bgr[..., 0]
    k = 1 - np.max([r, g, b], axis=0)
    c = (1 - r - k) / (1 - k + 1e-8)
    m = (1 - g - k) / (1 - k + 1e-8)
    y = (1 - b - k) / (1 - k + 1e-8)
    cmyk = np.stack((c, m, y, k), axis=-1) * 255
    return cmyk.astype(np.uint8)

def adjust_brightness_contrast(image):
    alpha = random.uniform(0.8, 1.2)
    beta = random.randint(-30, 30) 
    return cv2.convertScaleAbs(image, alpha=alpha, beta=beta)

def extract_HOG(image):
    winSize = (64, 64) 
    blockSize = (16, 16)
    blockStride = (8, 8)
    cellSize = (8, 8)
    nbins = 9
    hog = cv2.HOGDescriptor(winSize, blockSize, blockStride, cellSize, nbins)
    return hog.compute(image).flatten()

def extract_features_cymk(image):
    cmyk_img = rgb_to_cmyk(image)
    grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    c_features = extract_HOG(cv2.resize(cmyk_img[..., 0], (64, 64)))
    m_features = extract_HOG(cv2.resize(cmyk_img[..., 1], (64, 64)))
    y_features = extract_HOG(cv2.resize(cmyk_img[..., 2], (64, 64)))
    k_features = extract_HOG(cv2.resize(cmyk_img[..., 3], (64, 64)))

    gray_features = extract_HOG(cv2.resize(grayscale, (64, 64)))

    return np.concatenate((c_features, m_features, y_features, k_features, gray_features))


train_data = []
train_labels = []
for i in range(len(train_lines)):
    original_img = cv2.imread(imageDirectory + train_lines[i][0] + imageType)
    augmented_img = adjust_brightness_contrast(original_img)

    features = extract_features_cymk(augmented_img)
    train_data.append(features)
    train_labels.append(int(train_lines[i][1]))

train_data = np.array(train_data, dtype=np.float32)
train_labels = np.array(train_labels, dtype=np.int32)

test_data = []
test_labels = []
for i in range(len(test_lines)):
    original_img = cv2.imread(imageDirectory + test_lines[i][0] + imageType)
    features = extract_features_cymk(original_img)
    test_data.append(features)
    test_labels.append(int(test_lines[i][1]))

test_data = np.array(test_data, dtype=np.float32)
test_labels = np.array(test_labels, dtype=np.int32)

scaler = MinMaxScaler()
train_data = scaler.fit_transform(train_data)
test_data = scaler.transform(test_data)

selector = SelectKBest(score_func=f_classif, k=1500)
train_data = selector.fit_transform(train_data, train_labels)
test_data = selector.transform(test_data)

# svm = cv2.ml.SVM_create()
# svm.setType(cv2.ml.SVM_C_SVC)
# svm.setKernel(cv2.ml.SVM_RBF)
# svm.setC(100.0)
# svm.setGamma(0.001)
svm = SVC(kernel='rbf', C=100.0, gamma=0.001)
svm.fit(train_data, train_labels)

# svm.train(train_data, cv2.ml.ROW_SAMPLE, train_labels)

correct = 0
confusion_matrix = np.zeros((6, 6)) 

for i in range(len(test_lines)):
    test_sample = test_data[i].reshape(1, -1)
    test_label = test_labels[i]

    predicted_label = svm.predict(test_sample)[0]

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

scores = cross_val_score(svm, train_data, train_labels, cv=5, scoring='accuracy')
print(f"Cross-Validation accuracy just to make sure my model is not acting funny: {np.mean(scores) * 100:.2f}% (±{np.std(scores) * 100:.2f}%)")

##This is for debugging
# for i in range(len(test_lines)):
#     original_img = cv2.imread(imageDirectory + test_lines[i][0] + imageType)
#     test_img = cv2.resize(original_img, (64, 64))  # Resize to match feature extraction dimensions

#     test_sample = test_data[i].reshape(1, -1)
#     _, result = svm.predict(test_sample)
#     predicted_label = int(result[0, 0])
#     true_label = test_labels[i]

#     if predicted_label != true_label:
#         if __debug__:
#             cv2.imshow("Original Image", original_img)
#             cv2.imshow("Resized Image", test_img)
#             print(f"Misclassified: True Label = {true_label}, Predicted = {predicted_label}")
#             key = cv2.waitKey(0)  # Wait for a key press to show the next image
#             if key == 27:  # Press 'Esc' to exit visualization
#                 break

# cv2.destroyAllWindows()


