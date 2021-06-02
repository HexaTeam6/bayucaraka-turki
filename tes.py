import RPi.GPIO as GPIO
import time
import numpy as np
import math
import argparse


ap = argparse.ArgumentParser()
ap.add_argument("-d","--degree", type=int, default=10, help="Degree")
ap.add_argument("-p", "--pin", type=int, default=17, help="Pinout")
args = vars(ap.parse_args())

#v = 12
#h = 15
#a = np.arctan(np.sqrt(np.square(v)/(4.9*h)))
#a =  math.degrees(a)
#print(a)

servoPIN = args["pin"]
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)

p = GPIO.PWM(servoPIN, 100) # GPIO 17 for PWM with 50Hz
p.start(2.5) # Initialization

duty = (args["degree"]) / 10.0 + 2.5
p.ChangeDutyCycle(duty)
time.sleep(1.0)

p.stop()
GPIO.cleanup()

