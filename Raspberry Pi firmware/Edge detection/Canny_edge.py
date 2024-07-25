import cv2
import numpy as np

# Load the image
image = cv2.imread('input_image.jpeg', 0)  # Read image as grayscale

# Perform edge detection using Canny edge detector
edges = cv2.Canny(image, 50, 200)  # Adjust the thresholds as needed

# Display the original and edge-detected images
cv2.imshow('Original Image', image)
cv2.imshow('Edge Detection', edges)
cv2.waitKey(0)
cv2.destroyAllWindows()
