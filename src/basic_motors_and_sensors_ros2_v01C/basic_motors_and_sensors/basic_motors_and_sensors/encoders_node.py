#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
title: encoders_node Node - ME439 Intro to robotics, wisc.edu
Author: Peter Adamczyk 
Updated 2025-02-05

encoders_node.py
ROS2 Node to a continually read data from encoders and publish it on topics ("encoder_0" or other)   
"""


import rclpy
from rclpy.node import Node
from gpiozero import RotaryEncoder  
# "traceback" is a library that lets you track down errors. 
import traceback 
# Import the message types we will need
from std_msgs.msg import Int32


# Publish sensors data at the rate it comes in
# Here we publish: 
#    Encoders (E0 and E1, (in "counts"))
# Here we publish each in a separate topic. 
# In the long-term it would be better to publish them together.  

# This is the central code: set up a Node, set up Publisher(s)


class EncodersNode(Node): 
    def __init__(self): 
        super().__init__('encoders')
        
        # Encoders and Motors from GPIOZERO
        self.e0 = RotaryEncoder(10,9,max_steps=0,bounce_time=None)
        self.e1 = RotaryEncoder(23,24,max_steps=0,bounce_time=None)
        
        # Create the publishers. Name each topic "encoder_##", with message type "Int32" 
        # Note the queue_size=1 statement: don't let it develop a backlog! 
        
        # Example: E0
        self.publisher_enc0 = self.create_publisher(Int32, 'encoder_0', 1)
        self.publisher_enc1 = self.create_publisher(Int32, 'encoder_1', 1)
        
        self.timer = self.create_timer(0.5, self.update_encoders)
        
    def update_encoders(self):
        #read the encoders:
        leftEnc = self.e0.steps 
        rightEnc = self.e1.steps 
        
        e0msg = Int32()
        e1msg = Int32()
        e0msg.data = leftEnc
        e1msg.data = rightEnc
        
        self.publisher_enc0.publish(e0msg)
        self.publisher_enc1.publish(e1msg)
        
            
def main(args=None):
    rclpy.init(args=args)

    encoders_node_instance = EncodersNode()
    rclpy.spin(encoders_node_instance)
        


if __name__ == '__main__':
    try: 
        main()
    except :
        traceback.print_exc()
        pass
