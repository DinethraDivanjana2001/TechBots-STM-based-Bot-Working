import cv2
import numpy as np

# Load the image
image = cv2.imread('input_image.jpeg', 1)  # Read image as color

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

# Calculate the midpoint and draw detected squares on the original image
square_image = image.copy()
midpoints = []
for square in squares:
    M = cv2.moments(square)
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        midpoints.append((cX, cY))
        cv2.drawContours(square_image, [square], -1, (0, 255, 0), 2)

# Calculate the centroid of all detected squares
if midpoints:
    midpoint_x = sum(point[0] for point in midpoints) // len(midpoints)
    midpoint_y = sum(point[1] for point in midpoints) // len(midpoints)

    # Draw a point at the calculated centroid
    cv2.circle(square_image, (midpoint_x, midpoint_y), 5, (0, 0, 255), -1)

    # Show coordinates of the midpoint on the image
    cv2.putText(square_image, f"({midpoint_x}, {midpoint_y})", (midpoint_x + 10, midpoint_y - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

# Display the original image with detected squares and the midpoint point
cv2.imshow('Squares Detected', square_image)

# Display edge detection image
cv2.imshow('Edge Detection', edges)

cv2.waitKey(0)
cv2.destroyAllWindows()
