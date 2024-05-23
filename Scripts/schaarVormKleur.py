import cv2
import numpy as np

# Function to saturate the image
def saturate_image(hsv_image, saturation_scale):
    hsv_image[:, :, 1] = np.clip(hsv_image[:, :, 1] * saturation_scale, 0, 255)
    return hsv_image

# Function to count children of a given contour
def count_children(hierarchy, index):
    count = 0
    child = hierarchy[index][2]  # First child index
    while child != -1:
        count += 1
        child = hierarchy[child][0]  # Next child index
    return count

# Update threshold callback function (not needed in this case, but required by createTrackbar)
def update_threshold(x):
    pass

# Initialize video capture
cap = cv2.VideoCapture(0)

# Create a window and add a trackbar for threshold adjustment
cv2.namedWindow('Black Frame')
cv2.createTrackbar('Threshold', 'Black Frame', 140, 255, update_threshold)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.resize(frame, (800, 600))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    saturated_hsv = saturate_image(hsv, 1.5)
    
    # Define range of red color in HSV
    lower_red1 = np.array([0, 70, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 70, 70])
    upper_red2 = np.array([180, 255, 255])
    
    # Threshold the HSV image to get only red colors
    red_mask1 = cv2.inRange(saturated_hsv, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(saturated_hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    
    # Define range of magenta color in HSV
    lower_magenta = np.array([140, 50, 50])
    upper_magenta = np.array([170, 255, 255])
    
    # Threshold the HSV image to get only magenta colors
    magenta_mask = cv2.inRange(saturated_hsv, lower_magenta, upper_magenta)
    
    # Define range of blue color in HSV
    lower_blue = np.array([100, 70, 70])
    upper_blue = np.array([130, 255, 255])
    
    # Threshold the HSV image to get only blue colors
    blue_mask = cv2.inRange(saturated_hsv, lower_blue, upper_blue)
    combined_mask = cv2.bitwise_or(cv2.bitwise_or(red_mask, magenta_mask), blue_mask)
    colored_objects = cv2.bitwise_and(frame, frame, mask=combined_mask)
    
    # Convert frame to HSV color space
    saturated_BGR = cv2.cvtColor(saturated_hsv, cv2.COLOR_HSV2BGR)
    gray = cv2.cvtColor(saturated_BGR, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (25, 25), 0)
    
    # Get the current position of the threshold trackbar
    threshold_value = cv2.getTrackbarPos('Threshold', 'Black Frame')
    retRect, threshRect = cv2.threshold(blur, threshold_value, 255, cv2.THRESH_BINARY_INV)
    ding = cv2.bitwise_and(colored_objects, colored_objects, mask=threshRect)
    grayDing = cv2.cvtColor(ding, cv2.COLOR_BGR2GRAY)
    blurDing = cv2.GaussianBlur(grayDing, (5, 5), 0)
    retDing, threshDing = cv2.threshold(blurDing, 20, 255, cv2.THRESH_BINARY)
    contoursDing, hierarchy3 = cv2.findContours(threshDing, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    selected_contours = [c for c in contoursDing if cv2.contourArea(c) > 1000]
    
    # Draw selected contours with colors based on their masks
    for contour in selected_contours:
        mask = np.zeros_like(grayDing)
        cv2.drawContours(mask, [contour], -1, 255, -1)
        
        red_overlap = cv2.countNonZero(cv2.bitwise_and(red_mask, red_mask, mask=mask))
        magenta_overlap = cv2.countNonZero(cv2.bitwise_and(magenta_mask, magenta_mask, mask=mask))
        blue_overlap = cv2.countNonZero(cv2.bitwise_and(blue_mask, blue_mask, mask=mask))
        
        if red_overlap > magenta_overlap and red_overlap > blue_overlap:
            color = (0, 0, 255)  # Red
        elif magenta_overlap > red_overlap and magenta_overlap > blue_overlap:
            color = (255, 0, 255)  # Magenta
        else:
            color = (255, 0, 0)  # Blue
        
        cv2.drawContours(frame, [contour], -1, color, 3)
    
    # Display the images
    cv2.imshow('Original Frame', frame)
    cv2.imshow('Black Frame', threshRect)
    cv2.imshow('Ding Frame', ding)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
