import cv2
import numpy as np

# --- Updated HSV range for the green block ---
lower_green = np.array([60, 50, 30])  # Adjusted for better detection
upper_green = np.array([80, 255, 255])

# --- Define filtering parameters ---
MIN_BLOCK_AREA = 1000
ASPECT_RATIO_RANGE = (0.3, 3.0)
MIN_SOLIDITY = 0.7

def recognize_green_block(image):
    """
    Recognizes and highlights the green block in an input image.

    Args:
        image (numpy.ndarray): The input BGR image.

    Returns:
        numpy.ndarray: The original image with the green block highlighted.
    """
    try:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, lower_green, upper_green)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > MIN_BLOCK_AREA:
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w) / h
                if ASPECT_RATIO_RANGE[0] < aspect_ratio < ASPECT_RATIO_RANGE[1]:
                    hull = cv2.convexHull(contour)
                    hull_area = cv2.contourArea(hull)
                    solidity = float(area) / hull_area if hull_area > 0 else 0
                    if solidity > MIN_SOLIDITY:
                        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        return image
    except Exception as e:
        print(f"An error occurred: {e}")
        return image  # Return the original image even if an error occurs

if __name__ == "__main__":
    image_file = r"C:\Users\vercillg\WRO\WRO-FE-RocheReapers\wro\img\2test_screenshot_02.04.2025.png"

    img = cv2.imread(image_file)

    if img is not None:
        processed_image = recognize_green_block(img)

        cv2.imshow("Green Block Recognition", processed_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print(f"Error: Could not read image at {image_file}")
