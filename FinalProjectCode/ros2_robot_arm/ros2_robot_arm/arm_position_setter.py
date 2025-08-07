import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class ArmPositionSetter(Node):
    def __init__(self):
        super().__init__('arm_position_setter')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/target_position', 10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/object_position',
            self.object_position_callback,
            10)
        self.subscription  # prevent unused variable warning

    def object_position_callback(self, msg):
        # Forward the object position as the target position
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmPositionSetter()
    rclpy.spin(node)
    rclpy.shutdown()
