#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Sep 17 13:51:19 2024

@author: pi
"""


import rclpy
from rclpy.node import Node
import traceback

from std_msgs.msg import Float32


class MotorCommander(Node):
    
    def __init__(self):
        super().__init__('motor_commander')
        self.publisher_ = self.create_publisher(Float32, 'motor_command_left', 1)
        
    def operate(self):
        msg_out = Float32()
        try:
            while True:
                cmd = float(input('Enter left motor command:  ').strip())
                msg_out.data = cmd
                self.publisher_.publish(msg_out)
                self.get_logger().info('Publishing: %+5.3f' % msg_out.data)
        except:
            traceback.print_exc()
        

def main(args=None):
    rclpy.init(args=args)

    motor_commander_instance = MotorCommander()
    motor_commander_instance.operate()
        
    
if __name__ == '__main__':
    main()
    