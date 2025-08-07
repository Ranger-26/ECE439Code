import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
import time

SERIAL_PORT = 'COM4'
BAUD_RATE = 9600

class ServoCommandsNode(Node):
    def __init__(self):
        super().__init__('servo_commands')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/joint_angles',
            self.joint_angles_callback,
            10)
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2)
        except serial.SerialException:
            self.get_logger().error('Failed to connect to Arduino.')
            self.ser = None

    def joint_angles_callback(self, msg):
        if self.ser:
            beta1, beta2 = msg.data
            command = f"{beta1},{beta2}\n"
            self.ser.write(command.encode())
            self.get_logger().info(f"Sent: {command.strip()}")


def main(args=None):
    rclpy.init(args=args)
    node = ServoCommandsNode()
    rclpy.spin(node)
    if node.ser:
        node.ser.close()
    rclpy.shutdown()
