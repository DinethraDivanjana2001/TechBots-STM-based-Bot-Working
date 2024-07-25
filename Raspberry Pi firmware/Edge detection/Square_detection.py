import cv2
import numpy as np

# Load the image
image = cv2.imread('input_image.jpeg', 0)  # Read image as grayscale

# Perform edge detection using Canny edge detector
edges = cv2.Canny(image, 100, 200)  # Adjust the thresholds as needed

# Find contours in the edge-detected image
contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Filter out contours that approximate to a square shape
squares = []
for contour in contours:
    perimeter = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
    if len(approx) == 4:
        squares.append(approx)

# Draw detected squares on the original image
square_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
cv2.drawContours(square_image, squares, -1, (0, 255, 0), 2)

# Display the original image with detected squares
cv2.imshow('Squares Detected', square_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
