import tkinter as tk
from tkinter import ttk
import serial
import time
import numpy as np

# === Serial Setup ===
SERIAL_PORT = 'COM4'  # Change this to your Arduino's port
BAUD_RATE = 9600
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
except serial.SerialException:
    print("Failed to connect to Arduino.")
    ser = None

current_angles = (90, 90)  # Default angles for servos

# === Inverse Kinematics ===
def calculate_angles(x, y, z, L1, L2):
    x_original = x
    x = abs(x)
    d = np.sqrt(x**2 + y**2)

    # Check reachability
    if d > (L1 + L2):
        return (np.arctan2(y, x) * 180/np.pi, 90)

    delta = np.arccos((x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2))
   # if np.abs(cos_theta2) > 1:
  #      return None
    b2 = np.pi/2 + delta
    # Elbow Down and Up
   # theta2_down = np.arccos(cos_theta2)
    #theta2_up = -theta2_down
    psi = np.arctan2(y, x)
    phi = np.arccos((L2**2-d**2-L1**2)/(-2*d*L1))

    b1 = psi + phi


    # # Convert to degrees and round
    if x_original < 0:
        b1 = np.pi - b1
        b2 = np.pi - b2

    b1 = b1 * 180/np.pi
    b2 = b2 * 180/np.pi

    if abs(b1) < 1e-10:
        b1 = 0
    if abs(b2) < 1e-10:
        b2 = 0
    print(f"Beta1: {b1}, Beta2: {b2}")
    if b1 < 0 or b1 > 180 or b2 < 0 or b2 > 180:
        return None
    else:
        return(b1,b2)

    

def send_angles(x, y, z, L1, L2):
    result = calculate_angles(x, y, z, L1, L2)
    if result is None:
        print("No valid solution found for the given coordinates.")
        return
    beta1, beta2 = result
    print(f"Angles: beta1 = {int(beta1)}, beta2 = {int(beta2)}")
    try:
        if ser:
            command = f"{beta1},{beta2}\n"
            ser.write(command.encode())
            print(command.strip())
    except Exception as e:
        print(f"Error: {str(e)}")
    

# === GUI Logic ===
# def send_angles():
#     try:
#         x = float(entry_x.get())
#         y = float(entry_y.get())
#         z = float(entry_z.get())
#         length1 = float(entry_l1.get())
#         length2 = float(entry_l2.get())


#         b = calculate_angles(x, y, z, length1, length2)
#         if b is None:
#             output_label.config(text="No valid solution found for the given coordinates.")
#             return         
#         beta1, beta2 = b
#         output_label.config(text=f"Angles: beta1 = {int(beta1)}, beta2 = {int(beta2)}")

#         if ser:
#             command = f"{beta1},{beta2}\n"
#             ser.write(command.encode())
#             print(command.strip())
#     except Exception as e:
#         output_label.config(text=f"Error: {str(e)}")


if __name__ == '__main__':
    # === GUI Layout ===
    root = tk.Tk()
    root.title("Arm Angle Controller")

    ttk.Label(root, text="X (arm frame):").grid(row=0, column=0)
    ttk.Label(root, text="Y (arm frame):").grid(row=1, column=0)
    ttk.Label(root, text="Z (arm frame):").grid(row=2, column=0)
    ttk.Label(root, text="Length 1:").grid(row=3, column=0)
    ttk.Label(root, text="Length 2:").grid(row=4, column=0)

    entry_x = ttk.Entry(root)
    entry_y = ttk.Entry(root)
    entry_z = ttk.Entry(root)
    entry_l1 = ttk.Entry(root)
    entry_l2 = ttk.Entry(root)

    entry_x.grid(row=0, column=1)
    entry_y.grid(row=1, column=1)
    entry_z.grid(row=2, column=1)
    entry_l1.grid(row=3, column=1)
    entry_l2.grid(row=4, column=1)

    send_btn = ttk.Button(root, text="Send to Arduino", command=send_angles)
    send_btn.grid(row=5, column=0, columnspan=2, pady=10)

    output_label = ttk.Label(root, text="Enter arm frame coords and press Send")
    output_label.grid(row=6, column=0, columnspan=2)

    root.mainloop()

    