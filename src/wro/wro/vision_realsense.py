import pyrealsense2 as rs
import numpy as np
import cv2

try:
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert color frame to numpy array
        color_image = np.asanyarray(color_frame.get_data())
        display_image = color_image.copy()  # Erstelle eine Kopie für die Anzeige

        # Convert BGR to HSV for easier color detection
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # Define color ranges for red and green
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        lower_red_alt = np.array([170, 100, 100])
        upper_red_alt = np.array([180, 255, 255])
        lower_green = np.array([60, 30, 50])
        upper_green = np.array([80, 255, 255])

        # Create masks for red and green
        mask_red = cv2.inRange(hsv_image, lower_red, upper_red)
        mask_red_alt = cv2.inRange(hsv_image, lower_red_alt, upper_red_alt)
        mask_red = cv2.bitwise_or(mask_red, mask_red_alt)
        mask_green = cv2.inRange(hsv_image, lower_green, upper_green)

        # Kombiniere die Masken für beide Farben zur Erkennung
        combined_mask = cv2.bitwise_or(mask_red, mask_green)
        
        # Noise reduzieren (optional)
        combined_mask = cv2.erode(combined_mask, None, iterations=1)
        combined_mask = cv2.dilate(combined_mask, None, iterations=1)

        # Find contours for all objects
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Sortiere nach Größe (größtes zuerst)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        
        # Fokussiere auf das erste erkannte Objekt (das größte)
        if contours:
            # Nehme das größte Objekt
            largest_contour = contours[0]
            x, y, w, h = cv2.boundingRect(largest_contour)
            
            # Prüfe ob es rot oder grün ist
            mask_object = np.zeros_like(mask_red)
            cv2.drawContours(mask_object, [largest_contour], 0, 255, -1)
            
            # Bestimme ob das Objekt mehr rot oder mehr grün ist
            red_pixels = cv2.countNonZero(cv2.bitwise_and(mask_red, mask_object))
            green_pixels = cv2.countNonZero(cv2.bitwise_and(mask_green, mask_object))
            
            # Erstelle einen dicken Rahmen um das Objekt
            if red_pixels > green_pixels:
                # Rot
                cv2.rectangle(display_image, (x, y), (x + w, y + h), (0, 0, 255), 3)
                cv2.putText(display_image, "FOKUS: Rot", (x, y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # Optionale Hervorhebung des Objekts
                roi = color_image[y:y+h, x:x+w]
                rest = color_image.copy()
                rest = cv2.addWeighted(rest, 0.5, rest, 0, 0)  # Dunklere Version
                rest[y:y+h, x:x+w] = roi  # Original-Helligkeit für das Objekt
                display_image = rest
            else:
                # Grün
                cv2.rectangle(display_image, (x, y), (x + w, y + h), (0, 255, 0), 3)
                cv2.putText(display_image, "FOKUS: Grün", (x, y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Optionale Hervorhebung des Objekts
                roi = color_image[y:y+h, x:x+w]
                rest = color_image.copy()
                rest = cv2.addWeighted(rest, 0.5, rest, 0, 0)  # Dunklere Version
                rest[y:y+h, x:x+w] = roi  # Original-Helligkeit für das Objekt
                display_image = rest
            
            # Zeichne eine Zielmarkierung in der Mitte des Objekts
            center_x = x + w // 2
            center_y = y + h // 2
            cv2.drawMarker(display_image, (center_x, center_y), 
                          (255, 255, 255), cv2.MARKER_CROSS, 20, 2)
        
        # Zeichne für alle anderen Objekte einfache Rechtecke
        for contour in contours[1:]:  # Alle außer dem ersten
            x, y, w, h = cv2.boundingRect(contour)
            mask_object = np.zeros_like(mask_red)
            cv2.drawContours(mask_object, [contour], 0, 255, -1)
            
            red_pixels = cv2.countNonZero(cv2.bitwise_and(mask_red, mask_object))
            green_pixels = cv2.countNonZero(cv2.bitwise_and(mask_green, mask_object))
            
            if red_pixels > green_pixels:
                cv2.rectangle(display_image, (x, y), (x + w, y + h), (0, 0, 255), 1)
            else:
                cv2.rectangle(display_image, (x, y), (x + w, y + h), (0, 255, 0), 1)

        # Display the resulting image
        cv2.imshow('Fokus auf erstes Objekt', display_image)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()