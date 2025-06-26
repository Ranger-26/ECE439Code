#!/usr/bin/python3

"""
ME 439 Intro to robotics
title:lab 1 - DC Motors
"""

#------------imports----------------
import time
import numpy as np
from gpiozero import PhaseEnableMotor

# Set up the motors 
motor0 = PhaseEnableMotor(5,12)
motor1 = PhaseEnableMotor(6,13)


# Motor Demo -- dc - "duty cycle"
print("Ramp Up, 0 to 1.0 duty cycle forward\n")
for dc in np.arange(0,1.01,0.01):
    dc = np.clip(dc,-1.,1.)
    if dc>0:
        motor0.forward(dc)
        motor1.forward(dc)
    else:
        motor0.backward(-dc)
        motor1.backward(-dc)
    time.sleep(0.05)
    
print("Ramp Down, 1.0 duty cycle forward to 0 to 1.0 duty cycle backward\n")    
for dc in np.arange(1,-1.01,-0.01):
    dc = np.clip(dc,-1.,1.)
    if dc>0:
        motor0.forward(dc)
        motor1.forward(dc)
    else:
        motor0.backward(-dc)
        motor1.backward(-dc)
    time.sleep(0.05)
    
print("Ramp Up, 1.0 duty cycle forward to 0\n")    
for dc in np.arange(-1,0.01,0.01): # stops before it gets to 1
    dc = np.clip(dc,-1.,1.)
    if dc>0:
        motor0.forward(dc)
        motor1.forward(dc)
    else:
        motor0.backward(-dc)
        motor1.backward(-dc)
    time.sleep(0.05)
    
motor0.stop()
motor1.stop()

print( 'Final Duty Cycle: {0}'.format(0) )



