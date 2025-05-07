import cv2
import numpy as np

def recognize_nearest_blocks_with_depth(color_image_path, depth_image_path, min_block_area=500,
                                        green_saturation_thresh=50, green_value_thresh=50):
    """
    Recognizes the nearest colored blocks (red and green) in a color image
    using depth information.

    Args:
        color_image_path (str): Path to the color image.
        depth_image_path (str): Path to the depth image.
        min_block_area (int): Minimum area for a contour to be considered a block.
        green_saturation_thresh (int): Minimum saturation for green detection.
        green_value_thresh (int): Minimum value for green detection.

    Returns:
        tuple: A tuple containing:
            - dict: Dictionary of the nearest blocks {'red': {...}, 'green': {...}}.
            - numpy.ndarray: The color image with bounding boxes of the nearest blocks.
    """
    try:
        # Read the color and depth images
        color_img = cv2.imread(color_image_path)
        depth_img = cv2.imread(depth_image_path, cv2.IMREAD_GRAYSCALE)  # Assuming grayscale depth image
        if color_img is None or depth_img is None:
            print("Error: Could not open color or depth image.")
            return {}, None

        height, width = color_img.shape[:2]
        if depth_img.shape[:2] != (height, width):
            # Attempt to resize the depth image to match the color image
            depth_img = cv2.resize(depth_img, (width, height), interpolation=cv2.INTER_LINEAR)
            print("Warning: Depth image was resized to match color image dimensions.")


        output_img = color_img.copy()
        hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)

        red_blocks_depth = []
        green_blocks_depth = []

        # --- Red Block Detection ---
        lower_red_1 = np.array([0, 100, 100])
        upper_red_1 = np.array([10, 255, 255])
        lower_red_2 = np.array([170, 100, 100])
        upper_red_2 = np.array([180, 255, 255])
        mask_red_1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        mask_red_2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
        mask_red = cv2.bitwise_or(mask_red_1, mask_red_2)
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours_red:
            area = cv2.contourArea(contour)
            if area > min_block_area:
                x, y, w, h = cv2.boundingRect(contour)
                # Get the depth values within the bounding box
                depth_roi = depth_img[y:y + h, x:x + w]
                # Calculate a representative depth (e.g., minimum non-zero depth)
                valid_depths = depth_roi[depth_roi > 0]
                if valid_depths.size > 0:
                    min_depth = np.min(valid_depths)
                    M = cv2.moments(contour)
                    cX = int(M["m10"] / M["m00"]) if M["m00"] != 0 else -1
                    cY = int(M["m01"] / M["m00"]) if M["m00"] != 0 else -1
                    if cX != -1 and cY != -1:
                        red_blocks_depth.append({"depth": min_depth, "centroid": (cX, cY), "bounding_box": (x, y, w, h)})

        # --- Green Block Detection ---
        lower_green = np.array([40, green_saturation_thresh, green_value_thresh])
        upper_green = np.array([90, 255, 255])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours_green:
            area = cv2.contourArea(contour)
            if area > min_block_area:
                x, y, w, h = cv2.boundingRect(contour)
                # Get the depth values within the bounding box
                depth_roi = depth_img[y:y + h, x:x + w]
                # Calculate a representative depth (e.g., minimum non-zero depth)
                valid_depths = depth_roi[depth_roi > 0]
                if valid_depths.size > 0:
                    min_depth = np.min(valid_depths)
                    M = cv2.moments(contour)
                    cX = int(M["m10"] / M["m00"]) if M["m00"] != 0 else -1
                    cY = int(M["m01"] / M["m00"]) if M["m00"] != 0 else -1
                    if cX != -1 and cY != -1:
                        green_blocks_depth.append({"depth": min_depth, "centroid": (cX, cY), "bounding_box": (x, y, w, h)})

        nearest_blocks = {}

        # Find the nearest red block (smallest depth)
        if red_blocks_depth:
            nearest_red = min(red_blocks_depth, key=lambda item: item['depth'])
            nearest_blocks["red"] = {"centroid": nearest_red["centroid"], "bounding_box": nearest_red["bounding_box"]}
            x, y, w, h = nearest_red["bounding_box"]
            cv2.rectangle(output_img, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(output_img, "red", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Find the nearest green block (smallest depth)
        if green_blocks_depth:
            nearest_green = min(green_blocks_depth, key=lambda item: item['depth'])
            nearest_blocks["green"] = {"centroid": nearest_green["centroid"], "bounding_box": nearest_green["bounding_box"]}
            x, y, w, h = nearest_green["bounding_box"]
            cv2.rectangle(output_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(output_img, "green", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return nearest_blocks, output_img

    except Exception as e:
        print(f"An error occurred: {e}")
        return {}, None

if __name__ == "__main__":
        color_image_file = r"C:\Users\vercillg\WRO\WRO-FE-RocheReapers\wro\img\image3_Color.jpg"  # Replace with your color image path
        depth_image_file = r"C:\Users\vercillg\WRO\WRO-FE-RocheReapers\wro\img\image3_Depth_Depth.png"  # Replace with your depth image path
        min_area = 500
        green_saturation = 50
        green_value = 50
        nearest_blocks, output_image = recognize_nearest_blocks_with_depth(color_image_file, depth_image_file, min_area, green_saturation, green_value)

        if nearest_blocks:
                print("Nearest detected blocks (based on depth):")
                for color, block_info in nearest_blocks.items():
                        print(f"- Color: {color}, Centroid: {block_info['centroid']}")
        else:
                print("No blocks detected.")

        if output_image is not None:
                cv2.imshow("Detected Nearest Blocks (with Depth)", output_image)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
