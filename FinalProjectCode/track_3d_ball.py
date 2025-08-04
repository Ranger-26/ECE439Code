import cv2
import numpy as np

# ==== Load Calibration ====
left_data = np.load("single_camera_calibration/camera_calibration_data_left.npz")
right_data = np.load("single_camera_calibration/camera_calibration_data_right.npz")

mtxL, distL = left_data["mtx"], left_data["dist"]
mtxR, distR = right_data["mtx"], right_data["dist"]

# ==== Manual Extrinsics ====
# Right camera is `baseline` meters to the right of the left
baseline = 0.285  # meters — change to your measured distance
R = np.eye(3)
T = np.array([[baseline], [0], [0]])

P1 = mtxL @ np.hstack((np.eye(3), np.zeros((3,1))))
P2 = mtxR @ np.hstack((R, T))

# ==== Ball Detection Params ====
# Adjust these to your ball's color in HSV
lower_hsv = np.array([101, 100, 100])
upper_hsv = np.array([114, 255, 255])

def detect_ball_center(frame, lower, upper):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > 100:
            M = cv2.moments(largest)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return (cx, cy)
    return None

# ==== Setup Cameras ====
capL = cv2.VideoCapture(1,cv2.CAP_DSHOW)
capR = cv2.VideoCapture(0, cv2.CAP_DSHOW)

# Flip the right camera if it's physically upside down
flip_right = True

while True:
    retL, frameL = capL.read()
    retR, frameR = capR.read()

    if not retL or not retR:
        print("Failed to read from cameras.")
        break

    if flip_right:
        frameR = cv2.rotate(frameR, cv2.ROTATE_180)

    # Undistort
    undistL = cv2.undistort(frameL, mtxL, distL)
    undistR = cv2.undistort(frameR, mtxR, distR)

    # Detect ball
    ptL = detect_ball_center(undistL, lower_hsv, upper_hsv)
    ptR = detect_ball_center(undistR, lower_hsv, upper_hsv)

    # Triangulate if both points found
    if ptL and ptR:
        uL = np.array(ptL, dtype=np.float32).reshape(2, 1)
        uR = np.array(ptR, dtype=np.float32).reshape(2, 1)

        point4D = cv2.triangulatePoints(P1, P2, uL, uR)
        point4D /= point4D[3]

        X, Y, Z = point4D[0, 0], point4D[1, 0], point4D[2, 0]

        text = f"X={X:.2f}m Y={Y:.2f}m Z={Z:.2f}m"
        cv2.putText(undistL, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        cv2.circle(undistL, ptL, 5, (0, 0, 255), -1)
        cv2.circle(undistR, ptR, 5, (0, 0, 255), -1)

    # Show
    cv2.imshow("Left Camera", undistL)
    cv2.imshow("Right Camera", undistR)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

capL.release()
capR.release()
cv2.destroyAllWindows()
