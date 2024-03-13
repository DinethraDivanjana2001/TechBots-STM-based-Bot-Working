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
    # Create a copy of the frame in grayscale
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame_gray = cv2.cvtColor(frame_gray, cv2.COLOR_GRAY2BGR)  # Convert grayscale to BGR

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
    
    # Find contours in each color area
    contours_red, _ = cv2.findContours(cv2.cvtColor(red_area, cv2.COLOR_BGR2GRAY), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(cv2.cvtColor(green_area, cv2.COLOR_BGR2GRAY), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_blue, _ = cv2.findContours(cv2.cvtColor(blue_area, cv2.COLOR_BGR2GRAY), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Process red area
    if contours_red:
        red_contour = max(contours_red, key=cv2.contourArea)
        M = cv2.moments(red_contour)
        if M["m00"] != 0:
            red_center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            if red_center[0] < 3* frame.shape[1] / 8:
                cv2.putText(frame_gray, "left", (red_center[0], red_center[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            elif red_center[0] > 5* frame.shape[1] / 8:
                cv2.putText(frame_gray, "right", (red_center[0], red_center[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            else:
                cv2.putText(frame_gray, "middle", (red_center[0], red_center[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    
    # Process green area (similar to red)
    if contours_green:
        green_contour = max(contours_green, key=cv2.contourArea)
        M = cv2.moments(green_contour)
        if M["m00"] != 0:
            green_center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            if green_center[0] < 3* frame.shape[1] / 8:
                cv2.putText(frame_gray, "left", (green_center[0], green_center[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            elif green_center[0] > 5 * frame.shape[1] / 8:
                cv2.putText(frame_gray, "right", (green_center[0], green_center[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                cv2.putText(frame_gray, "middle", (green_center[0], green_center[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    # Process blue area (similar to red)
    if contours_blue:
        blue_contour = max(contours_blue, key=cv2.contourArea)
        M = cv2.moments(blue_contour)
        if M["m00"] != 0:
            blue_center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            if blue_center[0] < 3* frame.shape[1] / 8:
                cv2.putText(frame_gray, "left", (blue_center[0], blue_center[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            elif blue_center[0] > 5* frame.shape[1] / 8:
                cv2.putText(frame_gray, "right", (blue_center[0], blue_center[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            else:
                cv2.putText(frame_gray, "middle", (blue_center[0], blue_center[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
    
    # Display frame with colored texts
    cv2.imshow('Frame', frame_gray)



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
