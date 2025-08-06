import cv2
import numpy as np

# Globals
clicked_hsv = None
hsv_frame = None

# Initial HSV filter range (set to detect red by default)
lower_hsv = np.array([37, 100, 100])
upper_hsv = np.array([43, 255, 255])

def on_mouse(event, x, y, flags, param):
    global clicked_hsv, hsv_frame
    if event == cv2.EVENT_LBUTTONDOWN and hsv_frame is not None:
        clicked_hsv = hsv_frame[y, x]
        print(f"Clicked HSV: H={clicked_hsv[0]}, S={clicked_hsv[1]}, V={clicked_hsv[2]}")

def main():
    global hsv_frame
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    cv2.namedWindow("Frame")
    cv2.setMouseCallback("Frame", on_mouse)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Apply HSV mask
        mask = cv2.inRange(hsv_frame, lower_hsv, upper_hsv)

        # Optional morphological cleaning
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Show both original and mask
        cv2.imshow("Frame", frame)
        cv2.imshow("HSV Mask", mask)

        key = cv2.waitKey(1)
        if key == 27:  # ESC key
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
