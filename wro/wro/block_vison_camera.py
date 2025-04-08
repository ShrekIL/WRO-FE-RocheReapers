import cv2
import numpy as np

# --- Green HSV ranges ---
lower_green = np.array([60, 30, 50]) 
upper_green = np.array([80, 255, 255])

# --- Refined Red HSV ranges ---
lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 100, 100])
upper_red2 = np.array([180, 255, 255])

MIN_BLOCK_AREA = 1000
ASPECT_RATIO_RANGE = (0.3, 3.0)
MIN_SOLIDITY = 0.7
MIN_RED_SATURATION = 150 
MIN_RED_VALUE = 100

def recognize_blocks(image):
    found_blocks = []  # List to store detected blocks (color, center, width, height, area)
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
                        center_x = x + w // 2
                        center_y = y + h // 2
                        found_blocks.append({"color": "green", "center": (center_x, center_y), "width": w, "height": h, "area": area, "bounding_box": (x, y, w, h)})
                        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.circle(image, (center_x, center_y), 5, (0, 0, 0), -1)

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
                        mask = np.zeros_like(mask_red)
                        cv2.drawContours(mask, [contour], -1, 255, cv2.FILLED)
                        mean_saturation = np.mean(hsv[:, :, 1][mask == 255])
                        mean_value = np.mean(hsv[:, :, 2][mask == 255])
                        if mean_saturation > MIN_RED_SATURATION and mean_value > MIN_RED_VALUE:
                            center_x = x + w // 2
                            center_y = y + h // 2
                            found_blocks.append({"color": "red", "center": (center_x, center_y), "width": w, "height": h, "area": area, "bounding_box": (x, y, w, h)})
                            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
                            cv2.circle(image, (center_x, center_y), 5, (0, 0, 0), -1)

        return image, found_blocks
    except Exception as e:
        print(f"An error occurred: {e}")
        return image, []

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

        processed_frame, detected_blocks = recognize_blocks(frame)

        # Sort blocks by area
        detected_blocks.sort(key=lambda block: block["area"], reverse=True)

        if detected_blocks:
            closest_block = detected_blocks[0]
            print(f"Closest {closest_block['color']} block center: x={closest_block['center'][0]}, y={closest_block['center'][1]}, width={closest_block['width']}, height={closest_block['height']}")
            cv2.putText(processed_frame, f"Closest {closest_block['color']}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

            for block in detected_blocks[1:]: # Process other blocks (not prioritized in text)
                print(f"Other {block['color']} block center: x={block['center'][0]}, y={block['center'][1]}, width={block['width']}, height={block['height']}")

        cv2.imshow("Real-time Block Recognition", processed_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()