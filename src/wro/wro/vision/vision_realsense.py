import pyrealsense2 as rs
import numpy as np
import cv2

class ObjectDetector:
    def __init__(self, max_depth_threshold_m=1.0, depth_smooth_factor=0.005):
        self.max_depth_threshold_m = max_depth_threshold_m
        self.depth_smooth_factor = depth_smooth_factor

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        self.profile = self.pipeline.start(self.config)
        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def process_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return None, None, None

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0, 198, 114])
        upper_red = np.array([203, 255, 158])
        lower_green = np.array([73, 168, 52])
        upper_green = np.array([87, 255, 253])

        mask_red = cv2.inRange(hsv_image, lower_red, upper_red)
        mask_green = cv2.inRange(hsv_image, lower_green, upper_green)
        combined_mask = cv2.bitwise_or(mask_red, mask_green)

        combined_mask = cv2.erode(combined_mask, None, iterations=2)
        combined_mask = cv2.dilate(combined_mask, None, iterations=2)

        foreground_mask = (depth_image * self.depth_scale) < self.max_depth_threshold_m
        foreground_mask = foreground_mask.astype(np.uint8)

        depth_image_float = depth_image.astype(np.float32)
        depth_image_smoothed = cv2.bilateralFilter(depth_image_float, d=5, sigmaColor=50, sigmaSpace=100)
        foreground_mask_smoothed = (depth_image_smoothed * self.depth_scale) < self.max_depth_threshold_m
        foreground_mask_smoothed = foreground_mask_smoothed.astype(np.uint8)

        combined_mask = cv2.bitwise_and(combined_mask, foreground_mask_smoothed)
        background_mask = cv2.bitwise_not(combined_mask)

        blurred_background = cv2.GaussianBlur(color_image, (15, 15), 0)

        masked_background = cv2.bitwise_and(blurred_background, blurred_background, mask=background_mask)
        masked_objects = cv2.bitwise_and(color_image, color_image, mask=combined_mask)
        final_image = cv2.add(masked_background, masked_objects)

        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_object_area = 50
        valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_object_area]

        nearest_data = None

        for contour in valid_contours:
            x, y, w, h = cv2.boundingRect(contour)
            center_x = x + w // 2
            center_y = y + h // 2

            depth_region = depth_image[
                max(0, center_y - 2):min(depth_image.shape[0], center_y + 3),
                max(0, center_x - 2):min(depth_image.shape[1], center_x + 3)
            ]
            depth_region = depth_region[depth_region != 0]
            distance_m = np.mean(depth_region) * self.depth_scale if depth_region.size > 0 else 0

            if distance_m <= 0:
                continue

            mask_object = np.zeros_like(mask_red)
            cv2.drawContours(mask_object, [contour], 0, 255, -1)

            red_pixels = cv2.countNonZero(cv2.bitwise_and(mask_red, mask_object))
            green_pixels = cv2.countNonZero(cv2.bitwise_and(mask_green, mask_object))
            object_color = "RED" if red_pixels > green_pixels else "GREEN"

            if nearest_data is None or distance_m < nearest_data["distance_m"]:
                nearest_data = {
                    "color": object_color,
                    "x": x,
                    "y": y,
                    "w": w,
                    "h": h,
                    "center_x": center_x,
                    "center_y": center_y,
                    "distance_m": distance_m,
                }

        if nearest_data and nearest_data["distance_m"] <= 1.0:
            x = nearest_data["x"]
            y = nearest_data["y"]
            w = nearest_data["w"]
            h = nearest_data["h"]
            color = nearest_data["color"]
            distance = nearest_data["distance_m"]

            cv2.rectangle(final_image, (x, y), (x + w, y + h), (0, 255, 255), 2)
            cv2.putText(
                final_image,
                f"{color} | {round(distance, 2)} m",
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )
            cv2.circle(final_image, (nearest_data["center_x"], nearest_data["center_y"]), 5, (255, 0, 0), -1)

        return final_image, nearest_data, combined_mask

    def stop(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
