#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
title: sensors_node Node - ME439 Intro to robotics, wisc.edu
Author: Peter Adamczyk 
Updated 2024-09-19

sensors_node.py
ROS2 Node to a continually read data from a Serial port ('/dev/ttyUSB0' (Arduino Nano on USB) and publish it on a topic ("sensors_A0" or other)   
"""


import rclpy
from rclpy.node import Node
# Import "serial" to get data from the AlaMode
import serial   
# "traceback" is a library that lets you track down errors. 
import traceback 
# Import the message types we will need
from std_msgs.msg import Int32


# Publish sensors data at the rate it comes in
# Here we publish: 
#    Analog value of interest (levels) 
# 
# Could also publish Encoders (E0 and E1, (in "counts")), more Analogs (A0-A7, in "levels" 0-1023), Ultrasound (in microseconds till echo) readings, and Digital values of theArduino ports
# Here we publish each in a separate topic. 
# In the long-term it would be better to publish them together.  

# This is the central code: set up a Node, set up Publisher(s)


class ArduinoSensorStreamReader(Node): 
    def __init__(self):
        # Initialize a node called "sensors_node"
        super().__init__('sensors_node')
        
        # Create the publishers. Name each topic "sensors_##", with message type "Int32" 
        # (because that's what is received over the serial port)
        # Note the queue_size=1 statement: don't let it develop a backlog! 
        
        # Example: A0
        self.publisher_A0 = self.create_publisher(Int32, 'sensors_A0', 1)
        
        # Other examples: 
        # self.publisher_E0 = self.create_publisher(Int32, 'sensors_E0', 1)
        # self.publisher_U0 = self.create_publisher(Int32, 'sensors_U0', 1)
        # self.publisher_DB = self.create_publisher(Int32, 'sensors_DB', 1)
        


    def operate(self): 
        
        # Create message object to pack and send. 
        msg_A0 = Int32()        

        
        # Data come in on the Serial port. Set that up and start it. 
    
        #----------setup serial--------------
        ser = serial.Serial('/dev/ttyUSB0')  #serial port to alamode is /dev/ttyS0. # port to Arduino Nano is /dev/ttyUSB0 
        ser.baudrate = 57600 
        ser.bytesize = 8
        ser.parity = 'N'
        ser.stopbits = 1
        ser.timeout = 1 # one second time out. 
    
        ser.flush()  # Flush any data currently on the port
        ser.readline()
    
    
        # MAIN LOOP to keep loading the message with new data. 
        # NOTE that at the moment the data are coming from a separate thread, but this will be replaced with the serial port line reader in the future. 
        while True:
    
            try: 
                # Here we read the serial port for a string that looks like "e0:123456", which is an Encoder0 reading. 
                # When we get a reading, update the associated motor command
                line = ser.readline().decode().strip() #blocking function, will wait until read entire line
    #            print(line)
                line = line.split(":")
                # Element 0 of "line" will be a string that says what the data are: 
                data_type = line[0]
                # Element 1 of "line" will be the value, an Integer
                data_value = int(line[1])
    #            print(data_type)
    #            print(line)
                if data_type == 'A0':
                    msg_A0.data = data_value	# Analog reading 
                    self.publisher_A0.publish(msg_A0)
                else:
                    continue
                
            
            except Exception:
                print('Bad line received on Arduino port - ignoring and continuing.')
                pass


            
def main(args=None):
    rclpy.init(args=args)

    sensors_node_instance = ArduinoSensorStreamReader()
    sensors_node_instance.operate()
        


if __name__ == '__main__':
    try: 
        main()
    except :
        traceback.print_exc()
        pass
