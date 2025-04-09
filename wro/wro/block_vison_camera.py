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
    found_blocks = []  # List to store detected blocks (color, center, width, height, area, bounding_box)
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

def get_nearest_block_data(image):
    try:
        processed_frame, detected_blocks = recognize_blocks(image.copy())

        if not detected_blocks:
            print("No blocks detected in the image.")
            return None, processed_frame  # Return None and processed frame

        # Sort blocks by area to get the largest (assuming largest is nearest)
        detected_blocks.sort(key=lambda block: block["area"], reverse=True)

        nearest_block = detected_blocks[0]

        # You can choose which data you want to return
        nb = {
            "color": nearest_block["color"],
            "center": nearest_block["center"],
            "width": nearest_block["width"],
            "height": nearest_block["height"],
            "area": nearest_block["area"],
            "bounding_box": nearest_block["bounding_box"]
        }

        return nb["color"], nb["center"], (nb["width"], nb["height"]), processed_frame

    except Exception as e:
        print(f"An error occurred: {e}")
        return None, image  # Return None and original image

if __name__ == "__main__":
    # --- Capture a single image from the camera ---
    cap = cv2.VideoCapture(0)  # Use 0 for default camera, or the camera index

    if not cap.isOpened():
        print("Error: Could not open camera.")
        exit()

    ret, frame = cap.read()
    cap.release()  # Release the camera immediately after capturing

    if not ret:
        print("Error: Could not read frame from camera.")
        exit()

    nearest_block_data = get_nearest_block_data(frame)

    if nearest_block_data is not None:
        nearest_block_color, center, dimensions, processed_frame = nearest_block_data
        width, height = dimensions
        center_x, center_y = center

        if nearest_block_color:
            print(f"Nearest block color: {nearest_block_color}")
        print(f"Nearest block center (x, y): ({center_x}, {center_y})")
        print(f"Nearest block width: {width}")
        print(f"Nearest block height: {height}")

        # Optional: Display the image with only the nearest block highlighted
        try:
            if processed_frame is not None:
                bx, by, bw, bh = None, None, None, None
                for block in recognize_blocks(frame.copy())[1]:
                    if block["center"] == center:
                        bx, by, bw, bh = block["bounding_box"]
                        break

                if bx is not None:
                    color = (0, 255, 0) if nearest_block_color == "green" else (0, 0, 255)
                    cv2.rectangle(processed_frame, (bx, by), (bx + bw, by + bh), color, 2)
                    cv2.circle(processed_frame, center, 5, (0, 0, 0), -1)
                    cv2.putText(processed_frame, f"Nearest {nearest_block_color}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                    cv2.imshow("Block Recognition from Camera Image", processed_frame)

        except Exception as e:
            print(f"Error displaying image: {e}")
    else:
        cv2.imshow("Block Recognition from Camera Image", frame)

    cv2.waitKey(0)  # Wait until a key is pressed
    cv2.destroyAllWindows()