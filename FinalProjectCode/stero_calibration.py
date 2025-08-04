import cv2
import numpy as np
import os

# === CONFIG ===
chessboard_size = (9, 6)        # number of internal corners (cols, rows)
square_size = 0.0257            # real-world square size in meters (adjust yours)
num_required_images = 20
output_dir = "single_camera_calibration"
os.makedirs(output_dir, exist_ok=True)

# Prepare object points (0,0,0), (1,0,0), ..., scaled by square size
objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
objp[:, :2] = np.indices(chessboard_size).T.reshape(-1, 2)
objp *= square_size

objpoints = []  # 3D points in real-world space
imgpoints = []  # 2D points in image plane

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # Change to your camera ID
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("[INFO] Press 'c' to capture chessboard image when visible.")
print("[INFO] Press 'q' to quit and run calibration.")

while True:
    ret, frame = cap.read()
    frame = cv2.rotate(frame, cv2.ROTATE_180)  # Rotate frame if needed for better view
    if not ret:
        print("Failed to grab frame")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    display = frame.copy()
    if found:
        cv2.drawChessboardCorners(display, chessboard_size, corners, found)

    cv2.imshow("Camera", display)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('c') and found:
        objpoints.append(objp.copy())
        # Refine corner locations for better accuracy
        corners_subpix = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1),
                                          criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        imgpoints.append(corners_subpix)
        print(f"[INFO] Captured image #{len(objpoints)}")

    elif key == ord('q'):
        break

    if len(objpoints) >= num_required_images:
        print("[INFO] Required images captured.")
        break

cap.release()
cv2.destroyAllWindows()

if len(objpoints) == 0:
    print("No images captured, exiting.")
    exit(1)

print("[INFO] Calibrating camera...")
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("\nCamera matrix:\n", mtx)
print("\nDistortion coefficients:\n", dist.ravel())

# Calculate reprojection error
total_error = 0
for i in range(len(objpoints)):
    imgpoints_proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints_proj, cv2.NORM_L2) / len(imgpoints_proj)
    total_error += error

print(f"\nAverage reprojection error: {total_error / len(objpoints):.6f}")

# Save calibration data
np.savez(os.path.join(output_dir, "camera_calibration_data_right.npz"),
         mtx=mtx, dist=dist)

print(f"\n[INFO] Calibration data saved to: {output_dir}/camera_calibration_data.npz")
    