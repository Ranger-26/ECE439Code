import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
import numpy as np

# Import your IK logic from ik_test.py
from ik_test import calculate_angles

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/target_position',
            self.target_position_callback,
            10)
        self.publisher_ = self.create_publisher(Int32MultiArray, '/joint_angles', 10)
        self.L1 = 0.31
        self.L2 = 0.31

    def target_position_callback(self, msg):
        # msg.data = [x, y, z]
        x, y, z = msg.data
        result = calculate_angles(x, y, z, self.L1, self.L2)
        if result is not None:
            beta1, beta2 = result
            joint_msg = Int32MultiArray()
            joint_msg.data = [int(beta1), int(beta2)]
            self.publisher_.publish(joint_msg)


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()
    rclpy.spin(node)
    rclpy.shutdown()
