import cv2
import numpy as np

# --- Camera calibration parameters (Logitech C270)
K = np.array([
    [1430, 0.0, 480],
    [0.0, 1430, 620],
    [0.0, 0.0, 1.0]
])
baseline = 0.295  # meters

# Globals
hsv_frame_left = None
hsv_frame_right = None

def on_mouse(event, x, y, flags, param):
    global hsv_frame_left, hsv_frame_right
    if event == cv2.EVENT_LBUTTONDOWN:
        win_name = param
        if win_name == "Left":
            frame = hsv_frame_left
        else:
            frame = hsv_frame_right
        if frame is not None and y < frame.shape[0] and x < frame.shape[1]:
            pixel_hsv = frame[y, x]
            print(f"[{win_name} Camera] Clicked HSV: H={pixel_hsv[0]}, S={pixel_hsv[1]}, V={pixel_hsv[2]}")

def find_red_ball_center(hsv_frame, lower, upper):
    mask = cv2.inRange(hsv_frame, lower, upper)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > 200:
            (x, y), radius = cv2.minEnclosingCircle(largest)
            return int(x), int(y), int(radius), mask
    return None, None, None, mask

def estimate_depth(uL, uR, vL):
    fx = K[0, 0]
    cx = K[0, 2]
    cy = K[1, 2]

    disparity = uL - uR
    if disparity <= 0:
        return None, None, None

    Z = fx * baseline / disparity
    X = (uL - cx) * Z / fx
    Y = (vL - cy) * Z / fx  # Using fx approx for fy

    return X, Y, Z

def main():
    global hsv_frame_left, hsv_frame_right
    cap_left = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    cap_right = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap_left.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap_left.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
    cap_right.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap_right.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)

    if not cap_left.isOpened() or not cap_right.isOpened():
        print("Error: Could not open cameras.")
        return

    cv2.namedWindow('Left')
    cv2.namedWindow('Right')
    cv2.setMouseCallback('Left', on_mouse, param="Left")
    cv2.setMouseCallback('Right', on_mouse, param="Right")

    # Single HSV range for red (adjust if needed)
    lower_red = np.array([101, 50, 50])
    upper_red = np.array([115, 255, 255])

    while True:
        retL, frameL = cap_left.read()
        retR, frameR = cap_right.read()
        #frameR = cv2.flip(frameR, -1)  # Flip for better user interaction
        if not retL or not retR:
            break

        hsv_frame_left = cv2.cvtColor(frameL, cv2.COLOR_BGR2HSV)
        hsv_frame_right = cv2.cvtColor(frameR, cv2.COLOR_BGR2HSV)

        # Find red ball in both frames
        uL, vL, rL, maskL = find_red_ball_center(hsv_frame_left, lower_red, upper_red)
        uR, vR, rR, maskR = find_red_ball_center(hsv_frame_right, lower_red, upper_red)

        # Draw detections
        if uL is not None:
            cv2.circle(frameL, (uL, vL), rL, (0, 255, 0), 2)
        if uR is not None:
            cv2.circle(frameR, (uR, vR), rR, (0, 255, 0), 2)

        # Estimate and display depth
        if uL is not None and uR is not None:
            coords = estimate_depth(uL, uR, vL)
            if coords is not None:
                X, Y, Z = coords
                if X is not None and Y is not None and Z is not None:
                    text = f"3D Pos: X={X:.2f}m Y={Y:.2f}m Z={Z:.2f}m"
                    cv2.putText(frameL, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    print(text)

        cv2.imshow('Left', frameL)
        cv2.imshow('Right', frameR)
        cv2.imshow('Mask Left', maskL)
        cv2.imshow('Mask Right', maskR)

        key = cv2.waitKey(1)
        if key == 27:
            break

    cap_left.release()
    cap_right.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
