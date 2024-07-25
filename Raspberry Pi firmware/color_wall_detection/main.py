import cv2
import numpy as np

# Function to apply color filter
def apply_color_filter(frame, color_lower, color_upper):
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Threshold the HSV image to get only desired color
    mask = cv2.inRange(hsv, color_lower, color_upper)
    
    # Bitwise-AND mask and original image
    filtered = cv2.bitwise_and(frame, frame, mask=mask)
    
    return filtered

# Function to apply green and blue filter
def apply_filter(frame):
    # Split the frame into its RGB channels
    blue_channel, green_channel, red_channel = cv2.split(frame)

    # Set green channel to maximum intensity
    green_channel.fill(255)

    # Set blue channel to maximum intensity
    blue_channel.fill(255)

    # Merge the channels back together
    filtered_frame = cv2.merge((blue_channel, green_channel, red_channel))

    return filtered_frame

# Initialize the camera
cap = cv2.VideoCapture(0)

while True:
    # Read a frame from the camera
    ret, frame = cap.read()

    if not ret:
        break

    # Get the dimensions of the frame
    height, width, _ = frame.shape

    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame_gray = cv2.cvtColor(frame_gray, cv2.COLOR_GRAY2BGR)

    # Define the region of interest (middle portion of the frame)
    start_row, end_row = int(height * 0.25), int(height * 0.75)
    start_col, end_col = int(width * 0.25), int(width * 0.75)
    roi = frame[start_row:end_row, start_col:end_col]

    # # Apply the filter to the region of interest
    # filtered_roi = apply_filter(roi)

    # # Replace the original region of interest with the filtered region
    # frame[start_row:end_row, start_col:end_col] = filtered_roi

    lower_green = np.array([60, 100, 100])
    upper_green = np.array([90, 255, 255])

    green_area = apply_color_filter(roi, lower_green, upper_green)

    # Overlay green_area on the original frame
    frame_gray[start_row:end_row, start_col:end_col] = cv2.addWeighted(frame[start_row:end_row, start_col:end_col], 0.3, green_area, 1, 0)

    # Display the resulting frame
    cv2.imshow('Filtered Frame', frame_gray)

    # Check for key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
