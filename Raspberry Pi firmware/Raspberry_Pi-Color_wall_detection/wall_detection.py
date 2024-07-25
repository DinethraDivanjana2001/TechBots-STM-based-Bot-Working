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

# Function to calculate percentage of a color in ROI
def calculate_color_percentage(color_area, roi_area):
    color_pixels = cv2.countNonZero(cv2.cvtColor(color_area, cv2.COLOR_BGR2GRAY))
    total_pixels = cv2.countNonZero(cv2.cvtColor(roi_area, cv2.COLOR_BGR2GRAY))
    return (color_pixels / total_pixels) * 100

# Initialize the camera
cap = cv2.VideoCapture(0)

while True:
    # Read a frame from the camera
    ret, frame = cap.read()

    if not ret:
        break

    # Resize the frame to half its original resolution
    frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)

    # Get the dimensions of the frame
    height, width, _ = frame.shape

    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame_gray = cv2.cvtColor(frame_gray, cv2.COLOR_GRAY2BGR)

    # Define the region of interest (middle portion of the frame)
    start_row, end_row = int(height * 0.25), int(height * 0.75)
    start_col, end_col = int(width * 0.25), int(width * 0.75)
    roi = frame[start_row:end_row, start_col:end_col]

    # Apply green color filter
    lower_green = np.array([60, 100, 100])
    upper_green = np.array([90, 255, 255])
    green_area = apply_color_filter(roi, lower_green, upper_green)

    # Calculate green percentage
    green_percentage = calculate_color_percentage(green_area, roi)

    # Overlay green_area on the original frame
    frame_green = cv2.addWeighted(frame[start_row:end_row, start_col:end_col], 0.3, green_area, 1, 0)

    # Apply blue color filter
    lower_blue = np.array([90, 100, 100])
    upper_blue = np.array([130, 255, 255])
    blue_area = apply_color_filter(roi, lower_blue, upper_blue)

    # Calculate blue percentage
    blue_percentage = calculate_color_percentage(blue_area, roi)

    # Overlay blue_area on the original frame
    frame_blue = cv2.addWeighted(frame[start_row:end_row, start_col:end_col], 0.3, blue_area, 1, 0)
    if(green_percentage > 5 or blue_percentage > 5):
        if (green_percentage>blue_percentage):
            # Display green color in one window with percentage
            cv2.putText(frame_gray, f'Green: {green_percentage:.2f}%', (100, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            # Overlay green_area on the original frame
            frame_gray[start_row:end_row, start_col:end_col] = cv2.addWeighted(frame[start_row:end_row, start_col:end_col], 0.3, green_area, 1, 0)

        else:
            # Display blue color in another window with percentage
            cv2.putText(frame_gray, f'Blue: {blue_percentage:.2f}%', (100, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            # Overlay green_area on the original frame
            frame_gray[start_row:end_row, start_col:end_col] = cv2.addWeighted(frame[start_row:end_row, start_col:end_col], 0.3, blue_area, 1, 0)
    else:
        # Display blue color in another window with percentage
            cv2.putText(frame_gray, f'No Color Detected', (100, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 150, 255), 2)
            # Overlay green_area on the original frame
            frame_gray[start_row:end_row, start_col:end_col] = cv2.addWeighted(frame[start_row:end_row, start_col:end_col], 0.3, blue_area, 1, 0)
     # Display green color in one window
    cv2.imshow('Filtered Frame', frame_gray)
    # Check for key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
