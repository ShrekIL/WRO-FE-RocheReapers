import cv2 as cv
import numpy as np

def get_largest_contour(contours):
    if not contours:
        return None, 0, (0, 0)
    largest_contour = max(contours, key=cv.contourArea)
    area = cv.contourArea(largest_contour)
    M = cv.moments(largest_contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
    else:
        cx, cy = 0, 0
    return largest_contour, area, (cx, cy)


def detect_block(image):
    image = image[100:250, :]
    # Load the image
    
    # Brighten the image
    image = cv.convertScaleAbs(image, alpha=1.5, beta=30)
    
    # Apply Gaussian blur
    image = cv.GaussianBlur(image, (7, 7), 0)
    
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    
    # Define color ranges for red and green in HSV
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    
    lower_green = np.array([40, 40, 40])
    upper_green = np.array([80, 255, 255])
    
    # Create masks
    mask_red1 = cv.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv.inRange(hsv, lower_red2, upper_red2)
    mask_red = mask_red1 + mask_red2
    mask_green = cv.inRange(hsv, lower_green, upper_green)
    
    # Find contours
    contours_red, _ = cv.findContours(mask_red, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv.findContours(mask_green, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
    def get_largest_contour(contours):
        if not contours:
            return None, 0, (0, 0)
        largest_contour = max(contours, key=cv.contourArea)
        area = cv.contourArea(largest_contour)
        M = cv.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        else:
            cx, cy = 0, 0
        return largest_contour, area, (cx, cy)
    
    red_contour, red_area, red_center = get_largest_contour(contours_red)
    green_contour, green_area, green_center = get_largest_contour(contours_green)
    
    if red_area == 0 and green_area == 0:
        return None  # No block detected
    elif red_area > green_area:
        chosen_color = "red"
        chosen_contour = red_contour
        chosen_center = red_center
    else:
        chosen_color = "green"
        chosen_contour = green_contour
        chosen_center = green_center
    
    # Draw contour on the image
    cv.drawContours(image, [chosen_contour], -1, (0, 255, 0) if chosen_color == "green" else (0, 0, 255), 2)
    
    return chosen_color, chosen_center
