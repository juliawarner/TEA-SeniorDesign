# TEA@UCF Senior Design Mechatronics Project
# Central control file that lives on the Raspberry Pi
# Julia Warner

import RPi.GPIO as GPIO #used for controlling motors
import time
from adafruit_servokit import ServoKit #servo driver library
import board #used for controlling LED screens
import neopixel #used for controlling LED screens
import socket #socket library to receive controller commands

# **** GLOBAL VARIABLES ****
RUNNING = True
PERFORMANCE = True #use to switch between performance and drive mode


# **** ANIMATION FUNCTIONS ****

def happy():
    print("Happy")
    
def sad():
    print("Sad")
    
def confused():
    print("Confused")

def angry():
    print("Angry")

def yes():
    print("Yes")

#function to turn off
def all_LEDs_off():
    global pixels
    
    pixels.fill((0, 0, 0))
    
    pixels.show()
    
#ends program execution by changing RUNNING variable used in main function
def stop():
    global RUNNING #change global variable
    print("Stopping!")
    RUNNING = False

#beging to rotate torso by gently accelerating
def begin_torso_rotation(going_right):
    global m3_current_speed #change motor speed by altering global variable
    
    #set direction pins correctly
    if going_right:
        print("Accelerating to the right.")
        GPIO.output(m3_in1_pin,GPIO.HIGH)
        GPIO.output(m3_in2_pin,GPIO.LOW)
    else:
        print("Accelerating to the left.")
        GPIO.output(m3_in1_pin,GPIO.LOW)
        GPIO.output(m3_in2_pin,GPIO.HIGH)
        
    #accelerate from current speed to 100% speed
    while m3_current_speed < 100:
        m3_current_speed += 1
        m3_speed_pwm.ChangeDutyCycle(m3_current_speed)
        time.sleep(0.005) #sleep for 5 milliseconds between each speed increase
        
    print("Fully accelerated!")


#ends torso rotation by gradually decelerating
def end_torso_rotation():
    global m3_current_speed
    print("Slowing down")
    
    #slow down from current speed to 25% speed
    while m3_current_speed > 25:
        m3_current_speed -= 1
        m3_speed_pwm.ChangeDutyCycle(m3_current_speed)
        time.sleep(0.005) #sleep for 5 milliseconds between each speed increase
    
    #stop motor
    GPIO.output(m3_in1_pin,GPIO.LOW)
    GPIO.output(m3_in2_pin,GPIO.LOW)
    
    print("Torso rotation ended")
    

# **** MAIN FUNCTIONS ****

#switches modes between performance or drive
def switch_mode(performance):
    global PERFORMANCE
    
    #check if switching to performance or drive mode
    if performance and not PERFORMANCE:
        print("Switching to performance mode.")
        PERFORMANCE = True
    elif not performance and PERFORMANCE:
        print("Switching to drive mode.")
        PERFORMANCE = False

#When in performance mode, the Pi listens for input
#from the controller's computer and calls functions
#to trigger the appropriate animations
def performance_mode_loop():
    #check if new message received from Raspberry Pi
    try:
        raw_msg = connection.recv(1024)
    except socket.timeout:
        #socket timed out, no new message
        #return back to main function for next loop
        return
    else:
        #message found, process message
        msg = raw_msg.decode("utf-8")
        
        #execute correct function based on new message
        if msg == "x":
            stop()
        elif msg == "y":
            yes()
        elif msg == "h":
            happy()
        elif msg == "s":
            sad()
        elif msg == "a":
            angry()
        elif msg == "c":
            confused()
        elif msg == "tr":
            begin_torso_rotation(going_right=True)
        elif msg == "tl":
            begin_torso_rotation(going_right=False)
        elif msg == "ts":
            end_torso_rotation()
        else:
            connection.send(bytes("I don't know what you want from me!", "utf-8"))
            
        #check for message from Driver to switch to drive mode


#reads input coming from JoyCon
def drive_mode_loop():
    #check for input from JoyCon
    
    #call appropriate function
    
    #check for Performance Controller command to switch back to performance mode
    #check if new message received from Raspberry Pi
    try:
        raw_msg = connection.recv(1024)
    except socket.timeout:
        #socket timed out, no new message
        #return back to main function for next loop
        return
    else:
        #message found, process message
        msg = raw_msg.decode("utf-8")
        
        #enter performance mode if that was the command
        if msg == "p":
            switch_mode(performance=True)
        
        
def main():
    #loop until RUNNING variable is changed to false
    #RUNNING can be deactivated by a kill command from the controller
    while(RUNNING):
        #call function appropriate for the current mode
        if(PERFORMANCE):
            performance_mode_loop()
        else:
            drive_mode_loop()


# **** GPIO SETUP ****
print("Initializing GPIO connections")
GPIO.setmode(GPIO.BCM) #Use BCM numerotation mode

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
m3_current_speed = 25 #default speed is 25% of duty cycle (low)
m3_speed_pwm.start(m3_current_speed)

print("Motor 3 initialized.")


#LED eye screen

# Choose an open pin connected to the Data In of the NeoPixel strip, i.e. board.D18
# NeoPixels must be connected to D10, D12, D18 or D21 to work.
pixel_pin = board.D18
 
# The number of NeoPixels
num_pixels = 128
 
# The order of the pixel colors - RGB or GRB. Some NeoPixels have red and green reversed!
# For RGBW NeoPixels, simply change the ORDER to RGBW or GRBW.
ORDER = neopixel.GRB

#initialize GLOBAL object to encapsulate both LED screens
pixels = neopixel.NeoPixel(
    pixel_pin, num_pixels, brightness=0.2, auto_write=False, pixel_order=ORDER
)

print("LED screens initialized.")

print("Finished initializing GPIO connections!")


# **** SOCKET SETUP ****
print("Beginning socket setup.")

#constants for IP addresses and port number
RASPI_IP = '192.168.0.180'
PORT_NUM = 14359

#create socket object for self
rasPiSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

#bind socket to own IP address and random port
rasPiSocket.bind((RASPI_IP, PORT_NUM))

#set buffer
rasPiSocket.listen(5)

print("Waiting for connection from controller!")

#accept connection as new socket
connection, address = rasPiSocket.accept()

#set timeout to low number to create non-blocking socket behavior
connection.settimeout(0.25)

#print IP of controller computer
print(f"A connection to controller with IP {address} has been establised!")


# **** BLUETOOTH SETUP ****
print("Beginning setup of Bluetooth device.")

print("Bluetooth setup complete!")


print("Setup complete! Ready to receive commands!")

# **** END SETUP ****



#start listening for input
main()

#close socket connection when program ends
connection.close()

print("Connection closed, thank you!")

