from vision_realsense import ObjectDetector
import cv2

detector = ObjectDetector()

try:
    while True:
        frame, data, mask = detector.process_frame()
        if frame is not None:
            cv2.imshow("Camera View", frame)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC to exit
            break
finally:
    detector.stop()
