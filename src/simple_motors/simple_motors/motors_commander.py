#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 19 09:28:09 2025

@author: Team4
"""

import rclpy
from rclpy.node import Node
import traceback

from std_msgs.msg import Float32


class MotorCommander(Node):

    def __init__(self):
        super().__init__('motor_commander_right')
        self.publisher_ = self.create_publisher(Float32, 'motor_command_right', 1)

    def operate(self):
        msg_out = Float32()
        try:
            while True:
                cmd = float(input('Enter right motor command:  ').strip())
                msg_out.data = cmd
                self.publisher_.publish(msg_out)
                self.get_logger().info('Publishing: %+5.3f' % msg_out.data)
        except:
            traceback.print_exc()

def main(args=None):
    rclpy.init(args=args)

    motor_commander_instance = MotorCommander()
    motor_commander_instance.operate()
    # NOTE that this instance has an infinite loop in it, so we don't have to "spin" the node to keep it from ending.
#    rclpy.spin(motor_commander_instance)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
