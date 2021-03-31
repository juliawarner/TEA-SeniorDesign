# TEA@UCF Senior Design Mechatronics Project
# Central control file that lives on the Raspberry Pi
# Julia Warner

import RPi.GPIO as GPIO 
import time
from adafruit_servokit import ServoKit #servo driver library

# **** GPIO SETUP ****

#PCA9685 Servo Driver
#Connected to SDA and SCL pins
kit = ServoKit(channels=16)

#First L298N motor driver
#Connected to motor 1 and motor 2 (drive motors)
m1_in1_pin = 17 #forward
m1_in2_pin = 27 #backward
m1_speed_pin = 22
m2_in1_pin = 10 #forward
m2_in2_pin = 9 #backward
m2_speed_pin = 11

#Second L298N motor driver
#Connected to motor 3 (torso rotation motor)
m3_in1_pin = 5
m3_in2_pin = 6
m3_speed_pin = 26

#LED eye screen
led_pin = 23


