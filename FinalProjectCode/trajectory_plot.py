import cv2
import numpy as np
import pickle
import time
from ik_test import send_angles
from kalman_filter import KalmanFilter3DProjectile, predict_to_height

K = None 
dist = None
# --- Load calibration parameters ---
K = pickle.load(open("cam_matrix.p","rb"),encoding='bytes')
dist = pickle.load(open("dist_matrix.p","rb"),encoding='bytes')

print(K)
print(dist)

baseline = 0.305  # meters

# Globals
hsv_frame_left = None
hsv_frame_right = None


#prediction
positions = []

# --- Red color ranges in HSV (both ends of the hue wheel)
color_ranges = [
    (np.array([101, 100, 100]), np.array([114, 255, 255])),
]


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

def find_red_ball_center(hsv_frame, red_ranges):
    total_mask = np.zeros(hsv_frame.shape[:2], dtype=np.uint8)

    for lower, upper in red_ranges:
        mask = cv2.inRange(hsv_frame, lower, upper)
        total_mask = cv2.bitwise_or(total_mask, mask)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    total_mask = cv2.morphologyEx(total_mask, cv2.MORPH_OPEN, kernel)
    total_mask = cv2.morphologyEx(total_mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(total_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > 200:
            (x, y), radius = cv2.minEnclosingCircle(largest)
            return int(x), int(y), int(radius), total_mask
    return None, None, None, total_mask

def estimate_depth(uL, uR, vL):
    fx = K[0, 0]
    cx = K[0, 2]
    cy = K[1, 2]

    disparity = abs(uL - uR)
    if disparity == 0:
        return None, None, None

    Z = fx * baseline / disparity
    X = (uL - cx) * Z / fx
    Y = (vL - cy) * Z / fx  # Using fx approx for fy

    return X, Y, Z


def main():
    global hsv_frame_left, hsv_frame_right
    cap_left = cv2.VideoCapture(2, cv2.CAP_DSHOW)
    cap_right = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    cap_left.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap_left.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
    cap_right.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap_right.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
    cap_right.set(cv2.CAP_PROP_FPS, 30)
    cap_left.set(cv2.CAP_PROP_FPS, 30)

    if not cap_left.isOpened() or not cap_right.isOpened():
        print("Error: Could not open cameras.")
    last_frame_time = None
    total_time = 0.0
    while True:
        retL, frameL = cap_left.read()
        retR, frameR = cap_right.read()
        if not retL or not retR:
            break

        # Undistort frames
        frameL = cv2.undistort(frameL, K, dist)
        frameR = cv2.undistort(frameR, K, dist)

        hsv_frame_left = cv2.cvtColor(frameL, cv2.COLOR_BGR2HSV)
        hsv_frame_right = cv2.cvtColor(frameR, cv2.COLOR_BGR2HSV)

        uL, vL, rL, maskL = find_red_ball_center(hsv_frame_left, color_ranges)
        uR, vR, rR, maskR = find_red_ball_center(hsv_frame_right, color_ranges)

        if uL is not None:
            cv2.circle(frameL, (uL, vL), rL, (0, 255, 0), 2)
        if uR is not None:
            cv2.circle(frameR, (uR, vR), rR, (0, 255, 0), 2)

        current_time = time.time()
        delta_time = 0.0
        if last_frame_time is not None:
            delta_time = current_time - last_frame_time
        last_frame_time = current_time
        total_time += delta_time

        # Display delta time on frame
        dt_text = f"Delta Time: {delta_time*1000:.2f} ms"
        cv2.putText(frameL, dt_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        if uL is not None and uR is not None:
            coords = estimate_depth(uL, uR, vL)
            if coords is not None:
                X, Y, Z = coords
                if X is not None and Y is not None and Z is not None:
                    text = f"3D Pos: X={X:.2f}m Y={Y:.2f}m Z={Z:.2f}m"
                    cv2.putText(frameL, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                    print(text)
                    positions.append((X, Y, Z, total_time))
                    # if (len(positions) > 10):
                    #     final_pos = predict_to_height(positions,-0.45, 0.0166666667)[-1]
                    #     yArm = final_pos[2] - 0.736
                    #     xArm = final_pos[0] + baseline / 2
                    #     #print the original coordinates
                    #     print(f"Original Coordinates: X={final_pos[0]:.2f}m Y={final_pos[1]:.2f}m Z={final_pos[2]:.2f}m")            
                    #     print(f"Predicted Arm Coordinates: X={xArm:.2f}m Y={yArm:.2f}m")

        cv2.imshow('Left', frameL)
        cv2.imshow('Right', frameR)
        cv2.imshow('Mask Left', maskL)
        cv2.imshow('Mask Right', maskR)

        key = cv2.waitKey(1)
        if key == 27:  # ESC
            break
                    # if (len(positions) > 10):
                    #     final_pos = predict_to_height(positions,-0.45, 0.0166666667)[-1]
                    #     yArm = final_pos[2] - 0.736
                    #     xArm = final_pos[0] + baseline / 2
                    #     #print the original coordinates
                    #     print(f"Original Coordinates: X={final_pos[0]:.2f}m Y={final_pos[1]:.2f}m Z={final_pos[2]:.2f}m")            
                    #     print(f"Predicted Arm Coordinates: X={xArm:.2f}m Y={yArm:.2f}m")



        cv2.imshow('Left', frameL)
        cv2.imshow('Right', frameR)
        cv2.imshow('Mask Left', maskL)
        cv2.imshow('Mask Right', maskR)

        key = cv2.waitKey(1)
        if key == 27:  # ESC
            break


    cap_left.release()
    cap_right.release()
    cv2.destroyAllWindows()

    # Plot X, Y, Z vs time after window is closed
    if positions:
        import matplotlib.pyplot as plt
        times = [p[3] for p in positions]
        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]
        zs = [p[2] for p in positions]

        # Filter out points where any coordinate > 5 meters
        filtered = [(x, y, z, t) for x, y, z, t in zip(xs, ys, zs, times) if abs(x) <= 5 and abs(y) <= 5 and abs(z) <= 5]
        if filtered:
            import numpy as np
            from numpy.polynomial.polynomial import Polynomial
            import datetime
            import os
            f_times = np.array([p[3] for p in filtered])
            f_xs = np.array([p[0] for p in filtered])
            f_ys = np.array([p[1] for p in filtered])
            f_zs = np.array([p[2] for p in filtered])

            fig_f, axs_f = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
            # X
            axs_f[0].scatter(f_times, f_xs, color='r', label='Filtered', s=20)
            if len(f_times) >= 3:
                coefs_x = np.polyfit(f_times, f_xs, 2)
                fit_x = np.polyval(coefs_x, f_times)
                axs_f[0].plot(f_times, fit_x, 'k-', label='Poly Fit (deg 2)')
            axs_f[0].set_ylabel('X (m)')
            axs_f[0].set_title('X vs Time')
            axs_f[0].legend()
            # Y
            axs_f[1].scatter(f_times, f_ys, color='g', label='Filtered', s=20)
            if len(f_times) >= 3:
                coefs_y = np.polyfit(f_times, f_ys, 2)
                fit_y = np.polyval(coefs_y, f_times)
                axs_f[1].plot(f_times, fit_y, 'k-', label='Poly Fit (deg 2)')
            axs_f[1].set_ylabel('Y (m)')
            axs_f[1].set_title('Y vs Time')
            axs_f[1].legend()
            # Z
            axs_f[2].scatter(f_times, f_zs, color='b', label='Filtered', s=20)
            if len(f_times) >= 3:
                coefs_z = np.polyfit(f_times, f_zs, 2)
                fit_z = np.polyval(coefs_z, f_times)
                axs_f[2].plot(f_times, fit_z, 'k-', label='Poly Fit (deg 2)')
            axs_f[2].set_ylabel('Z (m)')
            axs_f[2].set_title('Z vs Time')
            axs_f[2].set_xlabel('Time (s)')
            axs_f[2].legend()

            plt.tight_layout()
            timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            folder = "plots"
            if not os.path.exists(folder):
                os.makedirs(folder)
            filename_filtered = os.path.join(folder, f"trajectory_plot_filtered_{timestamp}.png")
            fig_f.savefig(filename_filtered)
            print(f"Filtered plot with fit lines saved as {filename_filtered}")
            # --- Predicted vs Actual Plot ---
            # Predict X, Z at Y=0.45 using polynomial fit, plot vs actual
            if len(f_times) >= 3:
                # Fit Y vs time (quadratic), X/Z vs time (linear)
                coef_y = np.polyfit(f_times, f_ys, 2)
                coef_x = np.polyfit(f_times, f_xs, 1)
                coef_z = np.polyfit(f_times, f_zs, 1)
                # Find time where Y=0.45
                roots = np.roots([coef_y[0], coef_y[1], coef_y[2]-0.45])
                valid_roots = [r for r in roots if np.isreal(r) and f_times[0] <= r <= f_times[-1]]
                if valid_roots:
                    t_pred = np.real(valid_roots[0])
                    x_pred = np.polyval(coef_x, t_pred)
                    z_pred = np.polyval(coef_z, t_pred)
                    # Find actual closest to Y=0.45
                    actual_idx = np.argmin(np.abs(f_ys - 0.45))
                    actual_x = f_xs[actual_idx]
                    actual_z = f_zs[actual_idx]
                    actual_t = f_times[actual_idx]
                    # Plot predicted vs actual
                    fig_pa, ax_pa = plt.subplots(figsize=(8,6))
                    ax_pa.scatter(actual_t, actual_x, color='r', label='Actual X at Y=0.45')
                    ax_pa.scatter(t_pred, x_pred, color='b', label='Predicted X at Y=0.45')
                    ax_pa.scatter(actual_t, actual_z, color='g', label='Actual Z at Y=0.45')
                    ax_pa.scatter(t_pred, z_pred, color='m', label='Predicted Z at Y=0.45')
                    ax_pa.set_xlabel('Time (s)')
                    ax_pa.set_ylabel('Position (m)')
                    ax_pa.set_title('Predicted vs Actual X/Z at Y=0.45')
                    ax_pa.legend()
                    filename_pa = os.path.join(folder, f"predicted_vs_actual_{timestamp}.png")
                    fig_pa.savefig(filename_pa)
                    print(f"Predicted vs Actual plot saved as {filename_pa}")
                    plt.close(fig_pa)
            plt.show()


#plot all the points in 3d

if __name__ == "__main__":
    main()
