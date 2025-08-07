import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import tkinter as tk
from tkinter import ttk

class ServoGuiNode(Node):
    def __init__(self):
        super().__init__('servo_gui')
        self.publisher_ = self.create_publisher(Int32MultiArray, '/joint_angles', 10)
        self.root = tk.Tk()
        self.root.title("Servo Controller (ROS2)")
        frame = ttk.Frame(self.root, padding="20")
        frame.grid()
        ttk.Label(frame, text="Servo 1").grid(row=0, column=0, columnspan=3)
        self.servo1_entry_var = tk.StringVar(value="90")
        self.servo1_slider = ttk.Scale(
            frame, from_=0, to=180, orient='horizontal',
            command=lambda val: self.servo1_entry_var.set(str(int(float(val))))
        )
        self.servo1_slider.grid(row=1, column=0, columnspan=2, sticky="we")
        self.servo1_entry = ttk.Entry(frame, textvariable=self.servo1_entry_var, width=5)
        self.servo1_entry.grid(row=1, column=2)
        self.servo1_entry_var.trace_add("write", self.on_servo1_entry_change)
        ttk.Label(frame, text="Servo 2").grid(row=2, column=0, columnspan=3)
        self.servo2_entry_var = tk.StringVar(value="90")
        self.servo2_slider = ttk.Scale(
            frame, from_=0, to=180, orient='horizontal',
            command=lambda val: self.servo2_entry_var.set(str(int(float(val))))
        )
        self.servo2_slider.grid(row=3, column=0, columnspan=2, sticky="we")
        self.servo2_entry = ttk.Entry(frame, textvariable=self.servo2_entry_var, width=5)
        self.servo2_entry.grid(row=3, column=2)
        self.servo2_entry_var.trace_add("write", self.on_servo2_entry_change)
        confirm_button = ttk.Button(frame, text="Send to ROS2", command=self.send_servo_positions)
        confirm_button.grid(row=4, column=0, columnspan=3, pady=(10, 0))
        self.status_var = tk.StringVar(value="Ready")
        ttk.Label(frame, textvariable=self.status_var).grid(row=5, column=0, columnspan=3, pady=(10, 0))
        self.servo1_slider.set(90)
        self.servo2_slider.set(90)

    def on_servo1_entry_change(self, *args):
        try:
            val = int(self.servo1_entry_var.get())
            val = max(0, min(180, val))
            self.servo1_slider.set(val)
        except ValueError:
            pass

    def on_servo2_entry_change(self, *args):
        try:
            val = int(self.servo2_entry_var.get())
            val = max(0, min(180, val))
            self.servo2_slider.set(val)
        except ValueError:
            pass

    def send_servo_positions(self):
        try:
            angle1 = int(self.servo1_entry_var.get())
            angle2 = int(self.servo2_entry_var.get())
            msg = Int32MultiArray()
            msg.data = [angle1, angle2]
            self.publisher_.publish(msg)
            self.status_var.set(f"Sent: {angle1}, {angle2}")
        except ValueError:
            self.status_var.set("Invalid input")

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = ServoGuiNode()
    node.run()
    rclpy.shutdown()
