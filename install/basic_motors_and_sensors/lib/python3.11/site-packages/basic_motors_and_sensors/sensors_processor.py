#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
title: sensors_processor node - ME439 Intro to robotics, wisc.edu
Author: Peter Adamczyk 
Updated 2024-10-03

sensors_processor.py
ROS2 Node to a receive sensor data on a topic ("sensors_A0" or other) and process it, then publish the processed version to another topic ("sensors_A0_processed" or similar)  
"""


import rclpy
from rclpy.node import Node
# "traceback" is a library that lets you track down errors. 
import traceback
# Import the message types we will need
from std_msgs.msg import Int32, Float32


class SensorsProcessor(Node): 
    def __init__(self): 
        super().__init__('sensors_processor')
        
        # First create one or more publishers for the topics that will hold the processed data. 
        self.publisher_A0_proc = self.create_publisher(Float32, 'sensors_A0_proc', 1) 
        
        # Also create Subscriptions with callbacks to handle each of the inputs 
        self.subscriber_A0 = self.create_subscription(Int32, 'sensors_A0', self.cal_and_pub_A0, 1) 
        
        # Callback function, which will be called with incoming message data when messages are received by the Subscriber above. 
    def cal_and_pub_A0(self, msg_in): 
        # Unpack the message. 
        analog_level = msg_in.data
        
        # calibrate the relationship between the signal that comes in and the signal in real units
        # This can be from a Data Sheet, or from measurements you make yourself.
        
        # Example: MaxBotix LV-EZ02 says: 
        # Distance in Inches = Volts / (Vcc/512). 
        # For A0, Vcc is 3.3 Volts and signals are in 2^10 levels from 0 to Vcc
        # Therefore (Distance in Meters) = 0.0254 (m/in)*(distance in Inches)
        
    ##### UPDATE THESE EQUATIONS
        analog_volts = analog_level * (3.3/1024)
        distance_meters = analog_volts / (3.3/512) * 0.0254
    #####
        
        # Create a Message that will hold the out-going data
        A0_proc = Float32()
        
        # Pack the message with the processed data
        A0_proc.data = float(distance_meters)
        
        # Publish the newly packed Message
        self.publisher_A0_proc.publish(A0_proc)


def main(args=None):
    rclpy.init(args=args)
    sensors_processor_instance = SensorsProcessor()
    try:
        rclpy.spin(sensors_processor_instance)
    except : 
        traceback.print_exc()

    

if __name__ == '__main__':
    main() 
    
