import cv2
import numpy as np

# --- Green HSV ranges ---
lower_green = np.array([60, 50, 30])
upper_green = np.array([80, 255, 255])

# --- Refined Red HSV ranges ---
lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 100, 100])
upper_red2 = np.array([180, 255, 255])

# --- Filtering parameters ---
MIN_BLOCK_AREA = 1000
ASPECT_RATIO_RANGE = (0.3, 3.0)
MIN_SOLIDITY = 0.7
MIN_RED_SATURATION = 150  # Adjust as needed
MIN_RED_VALUE = 100  # adjust as needed

def recognize_green_block(image):
    found_green = False
    found_red = False
    try:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # --- Green detection ---
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours_green:
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
                        found_green = True

        # --- Red detection ---
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours_red:
            area = cv2.contourArea(contour)
            if area > MIN_BLOCK_AREA:
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w) / h
                if ASPECT_RATIO_RANGE[0] < aspect_ratio < ASPECT_RATIO_RANGE[1]:
                    hull = cv2.convexHull(contour)
                    hull_area = cv2.contourArea(hull)
                    solidity = float(area) / hull_area if hull_area > 0 else 0
                    if solidity > MIN_SOLIDITY:
                        # --- Check saturation and value ---
                        mask = np.zeros_like(mask_red)
                        cv2.drawContours(mask, [contour], -1, 255, cv2.FILLED)
                        mean_saturation = np.mean(hsv[:, :, 1][mask == 255])
                        mean_value = np.mean(hsv[:, :, 2][mask == 255])
                        if mean_saturation > MIN_RED_SATURATION and mean_value > MIN_RED_VALUE:
                            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
                            found_red = True

        return image, found_green, found_red
    except Exception as e:
        print(f"An error occurred: {e}")
        return image, found_green, found_red

if __name__ == "__main__":
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        exit()

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Error: Could not read frame.")
            break

        processed_frame, green_found, red_found = recognize_green_block(frame)

        if green_found:
            cv2.putText(processed_frame, "Left", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        elif red_found:
            cv2.putText(processed_frame, "Right", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

        cv2.imshow("Real-time Block Recognition", processed_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()