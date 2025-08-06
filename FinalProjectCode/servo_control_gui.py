import tkinter as tk
from tkinter import ttk
import serial
import time

# === Serial Configuration ===
SERIAL_PORT = 'COM4'  # Change this to your Arduino's port
BAUD_RATE = 9600

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
except serial.SerialException:
    print("Failed to connect to Arduino.")
    ser = None

# === Send to Arduino ===
def send_servo_positions():
    if ser is None:
        status_var.set("Serial not connected")
        return
    try:
        angle1 = int(servo1_entry_var.get())
        angle2 = int(servo2_entry_var.get())
        angle1 = max(0, min(180, angle1))
        angle2 = max(0, min(180, angle2))
        command = f"{angle1-8},{angle2+5}\n"
        ser.write(command.encode())
        status_var.set(f"Sent: {command.strip()}")
    except ValueError:
        status_var.set("Invalid input")

# === GUI Setup ===
root = tk.Tk()
root.title("Servo Controller (-20 to 200) with Confirm")

frame = ttk.Frame(root, padding="20")
frame.grid()

# === Servo 1 ===
ttk.Label(frame, text="Servo 1").grid(row=0, column=0, columnspan=3)

servo1_entry_var = tk.StringVar(value="90")
servo1_slider = ttk.Scale(
    frame, from_=0, to=180, orient='horizontal',
    command=lambda val: servo1_entry_var.set(str(int(float(val))))
)
servo1_slider.grid(row=1, column=0, columnspan=2, sticky="we")

servo1_entry = ttk.Entry(frame, textvariable=servo1_entry_var, width=5)
servo1_entry.grid(row=1, column=2)

def on_servo1_entry_change(*args):
    try:
        val = int(servo1_entry_var.get())
        val = max(0, min(180, val))
        servo1_slider.set(val)
    except ValueError:
        pass

servo1_entry_var.trace_add("write", on_servo1_entry_change)

# === Servo 2 ===
ttk.Label(frame, text="Servo 2").grid(row=2, column=0, columnspan=3)

servo2_entry_var = tk.StringVar(value="90")
servo2_slider = ttk.Scale(
    frame, from_=0, to=180, orient='horizontal',
    command=lambda val: servo2_entry_var.set(str(int(float(val))))
)
servo2_slider.grid(row=3, column=0, columnspan=2, sticky="we")

servo2_entry = ttk.Entry(frame, textvariable=servo2_entry_var, width=5)
servo2_entry.grid(row=3, column=2)

def on_servo2_entry_change(*args):
    try:
        val = int(servo2_entry_var.get())
        val = max(0, min(180, val))
        servo2_slider.set(val)
    except ValueError:
        pass

servo2_entry_var.trace_add("write", on_servo2_entry_change)

# === Confirm Button ===
confirm_button = ttk.Button(frame, text="Send to Arduino", command=send_servo_positions)
confirm_button.grid(row=4, column=0, columnspan=3, pady=(10, 0))

# === Status Display ===
status_var = tk.StringVar(value="Ready")
ttk.Label(frame, textvariable=status_var).grid(row=5, column=0, columnspan=3, pady=(10, 0))

# === Set Initial Values ===
servo1_slider.set(90)
servo2_slider.set(90)

# === Start GUI ===
root.mainloop()

if ser:
    ser.close()
