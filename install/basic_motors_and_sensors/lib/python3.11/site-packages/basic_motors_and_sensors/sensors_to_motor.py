#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
title: sensors_processor node - ME439 Intro to robotics, wisc.edu
Author: Team 5
Updated 2025-06-26

sensors_to_motor_command_node.py
ROS2 Node to a receive sensor data on a topic ("sensors_A0" or other) and process it, then publish the processed version to another topic ("sensors_A0_processed" or similar)  
"""


import rclpy
from rclpy.node import Node
# "traceback" is a library that lets you track down errors. 
import traceback
# Import the message types we will need
from std_msgs.msg import Int32, Float32


class SensorsToMotor(Node): 
    def __init__(self): 
        super().__init__('sensors_to_motor')
        
        # First create one or more publishers for the topics that will hold the processed data. 
        self.publisher_motor_left = self.create_publisher(Float32, 'motor_command_left', 1) 
        self.publisher_motor_right = self.create_publisher(Float32, 'motor_command_right', 1) 
        
        # Also create Subscriptions with callbacks to handle each of the inputs 
        self.subscriber_A0_proc = self.create_subscription(Int32, 'sensors_A0', self.cal_and_pub_A0, 1) 
	
	
        # Callback function, which will be called with incoming message data when messages are received by the Subscriber above. 
    def cal_and_pub_A0(self, msg_in): 
        # Unpack the message. 
        distance = msg_in.data
        msg_left = Float32()
        msg_right = Float32()

        if distance < 20:
            msg_left.data = -0.5
            msg_right.data = -0.5
        # Publish the newly packed Message
        else:
            msg_left.data = 0.0
            msg_right.data = 0.0
        
        self.publisher_motor_left.publish(msg_left)
        self.publisher_motor_right.publish(msg_right)

            
            
            


def main(args=None):
    rclpy.init(args=args)
    sensors_processor_instance = SensorsToMotor()
    try:
        rclpy.spin(sensors_processor_instance)
    except : 
        traceback.print_exc()

    

if __name__ == '__main__':
    main() 
    
