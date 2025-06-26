#!/usr/bin/env python3

import time
from gpiozero import PhaseEnableMotor

motor0 = PhaseEnableMotor(5,12)
motor1 = PhaseEnableMotor(6,13)

motor0.forward(0.3)
motor1.forward(0.3)

time.sleep(2)

motor0.stop()
motor1.stop()
