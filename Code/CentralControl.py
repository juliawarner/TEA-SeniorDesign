# TEA@UCF Senior Design Mechatronics Project
# Central control file that lives on the Raspberry Pi
# Julia Warner

import RPi.GPIO as GPIO 
import time
from adafruit_servokit import ServoKit #servo driver library

# **** GPIO SETUP ****
print("Initializing GPIO connections")

#PCA9685 Servo Driver
#Connected to SDA and SCL pins
#kit = ServoKit(channels=16)

print("Servo driver initialized.")

#First L298N motor driver
#Connected to motor 1 and motor 2 (drive motors)
m1_in1_pin = 17 #forward
m1_in2_pin = 27 #backward
m1_speed_pin = 22
GPIO.setup(m1_in1_pin,GPIO.OUT)
GPIO.setup(m1_in2_pin,GPIO.OUT)
GPIO.setup(m1_speed_pin,GPIO.OUT)
GPIO.output(m1_in1_pin,GPIO.LOW)
GPIO.output(m1_in2_pin,GPIO.LOW)
m1_speed_pwm = GPIO.PWM(m1_speed_pin,1000)
m1_speed_pwm.start(25) #default speed is 25% of duty cycle (low)

print("Motor 1 initialized.")

m2_in1_pin = 10 #forward
m2_in2_pin = 9 #backward
m2_speed_pin = 11
GPIO.setup(m2_in1_pin,GPIO.OUT)
GPIO.setup(m2_in2_pin,GPIO.OUT)
GPIO.setup(m2_speed_pin,GPIO.OUT)
GPIO.output(m2_in1_pin,GPIO.LOW)
GPIO.output(m2_in2_pin,GPIO.LOW)
m2_speed_pwm = GPIO.PWM(m2_speed_pin,1000)
m2_speed_pwm.start(25) #default speed is 25% of duty cycle (low)

print("Motor 2 initialized.")

#Second L298N motor driver
#Connected to motor 3 (torso rotation motor)
m3_in1_pin = 5
m3_in2_pin = 6
m3_speed_pin = 26
GPIO.setup(m3_in1_pin,GPIO.OUT)
GPIO.setup(m3_in2_pin,GPIO.OUT)
GPIO.setup(m3_speed_pin,GPIO.OUT)
GPIO.output(m3_in1_pin,GPIO.LOW)
GPIO.output(m3_in2_pin,GPIO.LOW)
m3_speed_pwm = GPIO.PWM(m3_speed_pin,1000)
m3_speed_pwm.start(25) #default speed is 25% of duty cycle (low)

print("Motor 3 initialized.")

#LED eye screen
led_pin = 23

print("LED screens initialized.")

print("Finished initializing GPIO connections!")

