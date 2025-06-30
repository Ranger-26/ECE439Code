#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Sep 17 14:03:20 2024

@author: pi
"""

import rclpy
from rclpy.node import Node
import traceback

from std_msgs.msg import Float32
from gpiozero import PhaseEnableMotor


class MotorExecutor(Node):

    def __init__(self):
        super().__init__('motor_executor')
        self.motor_left = PhaseEnableMotor(5,12)
        self.motor_right = PhaseEnableMotor(6,13)
        self.subscription_left = self.create_subscription(Float32, 'motor_command_left', self.set_motor_command_left, 1)        
        self.subscription_right = self.create_subscription(Float32,'motor_command_right', self.set_motor_command_right, 1)
    
    def set_motor_command_left(self, msg_in):
        motor_command_left = float(msg_in.data)
        self.get_logger().info('Received: %+5.3f' % msg_in.data)
        if motor_command_left >= 0:
            self.motor_left.forward(motor_command_left)
        else:
            self.motor_left.backward(-1*motor_command_left)
   
    def set_motor_command_right(self, msg_in):
        motor_command_right = float(msg_in.data)
        if motor_command_right >= 0:
    	    self.motor_right.forward(motor_command_right)
        else:
    	    self.motor_right.backward(-1*motor_command_right)
    
    def stop_motor_left(self):
        self.motor_left.stop()

    def stop_motor_right(self):
        self.motor_right.stop()

def main(args=None):
    rclpy.init(args=args)

    motor_executor = MotorExecutor()

    try:
        # "spin" will block the program until an error occurs, e.g. keyboard break Ctrl+C. 
        rclpy.spin(motor_executor)
    except:
        # When an error occurs, catch it with "except" and stop the motors
        motor_executor.stop_motor_left()
        motor_executor.stop_motor_right()
        motor_executor.get_logger().info('Stopping Motor') 

    rclpy.shutdown()


# Section to start the execution.  
if __name__ == "__main__":
    main()
