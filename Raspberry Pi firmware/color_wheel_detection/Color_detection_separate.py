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

# Function to display red, green, and blue areas
def display_color_areas(frame):
    # Define ranges for red, green, and blue colors in HSV
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])
    lower_green = np.array([50, 100, 100])
    upper_green = np.array([70, 255, 255])
    lower_blue = np.array([90, 100, 100])
    upper_blue = np.array([130, 255, 255])
    
    # Apply color filters
    red_area = apply_color_filter(frame, lower_red, upper_red)
    green_area = apply_color_filter(frame, lower_green, upper_green)
    blue_area = apply_color_filter(frame, lower_blue, upper_blue)
    
    # Display red, green, and blue areas
    cv2.imshow('Red Area', red_area)
    cv2.imshow('Green Area', green_area)
    cv2.imshow('Blue Area', blue_area)

# Main function
def main():
    # Capture video from camera
    cap = cv2.VideoCapture(0)
    
    while True:
        # Read a frame from the camera
        ret, frame = cap.read()
        
        if not ret:
            break
        
        # Display red, green, and blue areas
        display_color_areas(frame)
        
        # Exit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # Release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
