import cv2 as cv
import numpy as np

try:
    from .realsense_camera import RealSenseCamera
except:
    from realsense_camera import RealSenseCamera

cam = None

def init():
    global cam
    cam = RealSenseCamera()

def getBlock():
    hsv_red_min = [0, 198, 114]
    hsv_red_max = [203, 255, 158]

    hsv_green_min = [73, 168, 52]
    hsv_green_max = [87, 255, 253]


if __name__ == "__main__":
    init()

    res = getBlock()
    # Create a window
    cv.namedWindow("HSV Range Adjuster")

    nothing = lambda x: None

    # Create trackbars for H, S, V lower and upper range
    cv.createTrackbar("H Min", "HSV Range Adjuster", 0, 179, nothing)
    cv.createTrackbar("S Min", "HSV Range Adjuster", 0, 255, nothing)
    cv.createTrackbar("V Min", "HSV Range Adjuster", 0, 255, nothing)
    cv.createTrackbar("H Max", "HSV Range Adjuster", 179, 255, nothing)
    cv.createTrackbar("S Max", "HSV Range Adjuster", 255, 255, nothing)
    cv.createTrackbar("V Max", "HSV Range Adjuster", 255, 255, nothing)

    cv.createTrackbar("H Min 2", "HSV Range Adjuster", 0, 179, nothing)
    cv.createTrackbar("S Min 2", "HSV Range Adjuster", 0, 255, nothing)
    cv.createTrackbar("V Min 2", "HSV Range Adjuster", 0, 255, nothing)
    cv.createTrackbar("H Max 2", "HSV Range Adjuster", 0, 255, nothing)
    cv.createTrackbar("S Max 2", "HSV Range Adjuster", 0, 255, nothing)
    cv.createTrackbar("V Max 2", "HSV Range Adjuster", 0, 255, nothing)

    def draw_circle(event,x,y,flags,param):
        if event == cv.EVENT_LBUTTONDOWN:
            print(f"Mouse clicked at: {x}, {y}")
            # Get the HSV value at the clicked position
            hsv_value = hsv[y, x]
            print(f"HSV Value at clicked position: {hsv_value}")

    while True:
        cap = cam.capture()
        img = cap.color_image
        # --- Image Processing ---
        hsv = cv.cvtColor(cap.color_image, cv.COLOR_BGR2HSV)

        # Get current positions of trackbars
        h_min = cv.getTrackbarPos("H Min", "HSV Range Adjuster")
        s_min = cv.getTrackbarPos("S Min", "HSV Range Adjuster")
        v_min = cv.getTrackbarPos("V Min", "HSV Range Adjuster")
        h_max = cv.getTrackbarPos("H Max", "HSV Range Adjuster")
        s_max = cv.getTrackbarPos("S Max", "HSV Range Adjuster")
        v_max = cv.getTrackbarPos("V Max", "HSV Range Adjuster")

        h_min2 = cv.getTrackbarPos("H Min 2", "HSV Range Adjuster")
        s_min2 = cv.getTrackbarPos("S Min 2", "HSV Range Adjuster")
        v_min2 = cv.getTrackbarPos("V Min 2", "HSV Range Adjuster")
        h_max2 = cv.getTrackbarPos("H Max 2", "HSV Range Adjuster")
        s_max2 = cv.getTrackbarPos("S Max 2", "HSV Range Adjuster")
        v_max2 = cv.getTrackbarPos("V Max 2", "HSV Range Adjuster")

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])

        lower2 = np.array([h_min2, s_min2, v_min2])
        upper2 = np.array([h_max2, s_max2, v_max2])

        # Mask and result
        mask = cv.inRange(hsv, lower, upper)
        mask2 = cv.inRange(hsv, lower2, upper2)
        
        
        mask = cv.bitwise_or(mask, mask2)
        
        result = cv.bitwise_and(img, img, mask=mask)

        # Show the windows
        cv.imshow("Original", img)
        cv.imshow("Mask", mask)
        cv.imshow("Result", result)

        cv.setMouseCallback('Original',draw_circle)
        cv.setMouseCallback('Result',draw_circle)

        if cv.waitKey(50) & 0xFF == 27:  # ESC to exit
            print("HSV Ranges:")
            print(f"Color 1: {lower.tolist()} - {upper.tolist()}")
            print(f"Color 2: {lower2.tolist()} - {upper2.tolist()}")
            break

    cv.destroyAllWindows()
