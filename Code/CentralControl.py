# TEA@UCF Senior Design Mechatronics Project
# Central control file that lives on the Raspberry Pi
# Julia Warner

import RPi.GPIO as GPIO #used for controlling motors
import time
from adafruit_servokit import ServoKit #servo driver library
import board #used for controlling LED screens
import neopixel #used for controlling LED screens
import socket #socket library to receive controller commands
from evdev import InputDevice, categorize, ecodes #used for JoyCon Events

# **** GLOBAL VARIABLES ****
RUNNING = True
PERFORMANCE = True #use to switch between performance and drive mode
JOYCON_PATH = '/dev/input/event1'
#constants for IP addresses and port number
RASPI_IP = '192.168.0.180'
PORT_NUM = 14357
MOVE_TO_NEUTRAL_DELAY = 1 #wait in seconds used when homing servos

#**** SERVO NEUTRAL POSITIONS ****
#resting position in degrees for each servo
#up/down indicates if increasing/decreasing the value keeps the servo within its range of motion
#right arm
#right shoulder rotation
SERVO0_NEUTRAL = 156 #down
SERVO0_INDEX = 0
#right shoulder flexation
SERVO1_NEUTRAL = 157 #down
SERVO1_INDEX = 1
#right elbow flexation
SERVO2_NEUTRAL = 38 #up
SERVO2_INDEX = 2
#right gripper open/close
SERVO3_NEUTRAL = 82 #up
SERVO3_INDEX = 3

#left arm
#left shoulder rotation
SERVO4_NEUTRAL = 41 #up
SERVO4_INDEX = 4
#left shoulder flexation
SERVO5_NEUTRAL = 48 #up
SERVO5_INDEX = 5
#left elbow flexation
SERVO6_NEUTRAL = 133 #down
SERVO6_INDEX = 6
#left gripper open/close
SERVO7_NEUTRAL = 179 #down
SERVO7_INDEX = 7

#head tilt
#NEVER DO SERVO 8 LESS THAN 5
SERVO8_NEUTRAL = 45 #up = tilt head left, down = tilt head right
SERVO8_INDEX = 8
#head nod
#SERVO 9 SHOULD NEVER GO BELOW 90
SERVO9_NEUTRAL = 180
SERVO9_INDEX = 9
#need to track eyebrow positions to switch between any 2 facial expressions
servo8_current_pos = 45
servo9_current_pos = 180

#right eyebrow
SERVO10_NEUTRAL = 90 #down = tilt eyebrow down
SERVO10_INDEX = 10
#left eyebrow 
SERVO11_NEUTRAL = 90 #up = tilt eyebrow down
SERVO11_INDEX = 11
#need to track eyebrow positions to switch between any 2 facial expressions
servo10_current_pos = 90
servo11_current_pos = 90

#Parameters for max/min PWM impulses
MIN_IMP  =[500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500]
MAX_IMP  =[2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500]



# **** SERVO FUNCTIONS ****

# initializes the servos by setting min and max PWM impulse sizes
def initialize_servos():
    global servos

    for i in range(16):
        servos.servo[i].set_pulse_width_range(MIN_IMP[i] , MAX_IMP[i])
        servos.servo[i].actuation_range = 180

#disables the servo channels after we're done using them
#so they don't pull amperage
def disable_channels(channel_indicies):
    global servos
    
    for servo_num in channel_indicies:
        servos.servo[servo_num].angle = None

#moves a group of servos from their start position to their end position
#all at the same time
#servo_indices = array of nums for index of servo in servos list
#servo_starts = starting angle for each servo
#servo_ends = end angle for each servo
#speed = delay in seconds between each 1 degree change in servo angle
def move_servos(servo_indices, servo_starts, servo_ends, speed):
    global servos
    
    num_servos = len(servo_indices)
    
    #find the maximum angle change
    #also generate increasing/decreasing array
    max_diff = 0
    increasing = []
    for i in range(num_servos):
        diff = servo_starts[i] - servo_ends[i]
        
        #determine if increasing or decreasing
        if diff < 0:
            increasing.append(True)
        else:
            increasing.append(False)
            
        #check against max difference
        diff = abs(diff)
        if diff > max_diff:
            max_diff = diff
            
    #generate array with angle value for each step for each servo
    #start with starting angle
    angle_arrs = [[servo_starts[0]]]
    for i in range(1, num_servos):
        angle_arrs.append([servo_starts[i]])
        
    print(angle_arrs)
    
    #for each servo...
    for servo_num in range(num_servos):
        #generate array of each angle timestep
        #generate values until the servo with the greatest difference is reached
        for angle_step in range(1, max_diff + 1):
            #determine if we've already reached the end
            if angle_arrs[servo_num][angle_step - 1] == servo_ends[servo_num]:
                angle_arrs[servo_num].append(servo_ends[servo_num])
                continue; #move on to next step
            
            #haven't reach the end, increment by 1
            #determine increase or decrease
            if increasing[servo_num]:
                angle_arrs[servo_num].append(angle_arrs[servo_num][angle_step - 1] + 1)
            else:
                angle_arrs[servo_num].append(angle_arrs[servo_num][angle_step - 1] - 1)
    
    print(angle_arrs)
    
    #NOW we can acutally move the servos
    #for each angle change in the generated arrays...
    for angle_step in range(max_diff + 1):
        #for each servo...
        for servo_num in range(num_servos):
            #update angle
            target_servo = servo_indices[servo_num]
            target_angle = angle_arrs[servo_num][angle_step]
            servos.servo[target_servo].angle = target_angle
            print(f"Servo {servo_indices[servo_num]} at angle {servos.servo[servo_indices[servo_num]].angle}")
            #print(f"Target was {angle_arrs[servo_num][angle_step]}")
        
        #wait for the correct amount of time
        time.sleep(speed)

#moves each servo one at a time to it's pre-determined neutral position
def move_to_neutral():
    #specify we will be editing global variable
    global servos

    #moves all servos to neutral position 1 at a time, with time in between to settle
    print("Setting servos to neutral position")

    servos.servo[SERVO0_INDEX].angle = SERVO0_NEUTRAL
    time.sleep(MOVE_TO_NEUTRAL_DELAY)
    print("Servo 0 at neutral position")
    servos.servo[SERVO0_INDEX].angle = None #disable channel

    servos.servo[SERVO1_INDEX].angle = SERVO1_NEUTRAL
    time.sleep(MOVE_TO_NEUTRAL_DELAY)
    print("Servo 1 at neutral position")
    servos.servo[SERVO1_INDEX].angle = None #disable channel

    servos.servo[SERVO2_INDEX].angle = SERVO2_NEUTRAL
    time.sleep(MOVE_TO_NEUTRAL_DELAY)
    print("Servo 2 at neutral position")
    servos.servo[SERVO2_INDEX].angle = None #disable channel

    servos.servo[SERVO3_INDEX].angle = SERVO3_NEUTRAL
    time.sleep(MOVE_TO_NEUTRAL_DELAY)
    print("Servo 3 at neutral position")
    servos.servo[SERVO3_INDEX].angle = None #disable channel
    
    servos.servo[SERVO4_INDEX].angle = SERVO4_NEUTRAL
    time.sleep(MOVE_TO_NEUTRAL_DELAY)
    print("Servo 4 at neutral position")
    servos.servo[SERVO4_INDEX].angle = None #disable channel
    
    servos.servo[SERVO5_INDEX].angle = SERVO5_NEUTRAL
    time.sleep(MOVE_TO_NEUTRAL_DELAY)
    print("Servo 5 at neutral position")
    servos.servo[SERVO5_INDEX].angle = None #disable channel
    
    servos.servo[SERVO6_INDEX].angle = SERVO6_NEUTRAL
    time.sleep(MOVE_TO_NEUTRAL_DELAY)
    print("Servo 6 at neutral position")
    servos.servo[SERVO6_INDEX].angle = None #disable channel
    
    servos.servo[SERVO7_INDEX].angle = SERVO7_NEUTRAL
    time.sleep(MOVE_TO_NEUTRAL_DELAY)
    print("Servo 7 at neutral position")
    servos.servo[SERVO7_INDEX].angle = None #disable channel
    
    servos.servo[SERVO8_INDEX].angle = SERVO8_NEUTRAL
    time.sleep(2)
    print("Servo 8 at neutral position")
    servos.servo[SERVO8_INDEX].angle = None #disable channel
    
    servos.servo[SERVO9_INDEX].angle = SERVO9_NEUTRAL
    time.sleep(MOVE_TO_NEUTRAL_DELAY)
    print("Servo 9 at neutral position")
    servos.servo[SERVO9_INDEX].angle = None #disable channel
    
    servos.servo[SERVO10_INDEX].angle = SERVO10_NEUTRAL
    time.sleep(MOVE_TO_NEUTRAL_DELAY)
    print("Servo 10 at neutral position")
    servos.servo[SERVO10_INDEX].angle = None #disable channel
    
    servos.servo[SERVO11_INDEX].angle = SERVO11_NEUTRAL
    time.sleep(MOVE_TO_NEUTRAL_DELAY)
    print("Servo 11 at neutral position")
    servos.servo[SERVO11_INDEX].angle = None #disable channel

#tilts eyebrows down like he's angry
def angry_eyebrows():
    global servo10_current_pos
    global servo11_current_pos
    
    #animation keyframes
    right_eyebrow_down = 60
    left_eyebrow_down = 120
    
    #speed
    eyebrow_speed = 0.003 # 3 milliseconds
    
    print("Moving eyebrows to angry")
    
    move_servos([SERVO10_INDEX, SERVO11_INDEX],
                [servo10_current_pos, servo11_current_pos],
                [right_eyebrow_down, left_eyebrow_down],
                eyebrow_speed)
    
    print("Eyebrows angry")
    
    #update global variables
    servo10_current_pos = right_eyebrow_down
    servo11_current_pos = left_eyebrow_down

#moves eyebrows to neutral position and disables the channel
def neutral_eyebrows():
    global servo10_current_pos
    global servo11_current_pos
    
    #speed
    eyebrow_speed = 0.003 # 3 milliseconds
    
    print("Moving eyebrows to neutral")
    
    move_servos([SERVO10_INDEX, SERVO11_INDEX],
                [servo10_current_pos, servo11_current_pos],
                [SERVO10_NEUTRAL, SERVO11_NEUTRAL],
                eyebrow_speed)
    
    print("Eyebrows neutral")
    
    #update global variables
    servo10_current_pos = SERVO10_NEUTRAL
    servo11_current_pos = SERVO11_NEUTRAL

    #disable servo channels
    disable_channels([SERVO10_INDEX, SERVO11_INDEX])
    
#tilts eyebrows down like he's sad
def sad_eyebrows():
    global servo10_current_pos
    global servo11_current_pos
    
    #animation keyframes
    right_eyebrow_down = 75
    left_eyebrow_down = 105
    
    #speed
    eyebrow_speed = 0.003 # 3 milliseconds
    
    print("Moving eyebrows to sad")
    
    move_servos([SERVO10_INDEX, SERVO11_INDEX],
                [servo10_current_pos, servo11_current_pos],
                [right_eyebrow_down, left_eyebrow_down],
                eyebrow_speed)
    
    print("Eyebrows sad")
    
    #update global variables
    servo10_current_pos = right_eyebrow_down
    servo11_current_pos = left_eyebrow_down
    
#tilts one eyebrow up like he's confused
def confused_eyebrows():
    global servo11_current_pos
    
    #animation keyframes
    left_eyebrow_up = 60
    
    #speed
    eyebrow_speed = 0.003 # 3 milliseconds
    
    print("Moving eyebrows to confused")
    
    move_servos([SERVO11_INDEX],
                [servo11_current_pos],
                [left_eyebrow_up],
                eyebrow_speed)
    
    print("Eyebrows confused")
    
    #update global variables
    servo11_current_pos = left_eyebrow_up

#waves right arm
def wave_right_arm():
    global servos
    
    print("Starting wave! Hello!")
    
    #animation keyframe degrees
    shoulder_flexation = 95 #arm rasied to wave
    elbow_flexation_1 = 108 #starting point for wave
    elbow_flexation_2 = 128 #ending point for wave
    gripper_open = 102 #open position for gripper

    #animation speeds, delay in seconds between each change in servo position
    arm_raise_lower_speed = 0.010 #10 milliseconds
    wave_speed = 0.010 #10 milliseconds
    
    print("Raising arm!")
    
    move_servos([SERVO1_INDEX, SERVO2_INDEX, SERVO3_INDEX],
                [SERVO1_NEUTRAL, SERVO2_NEUTRAL, SERVO3_NEUTRAL],
                [shoulder_flexation, elbow_flexation_1, gripper_open],
                arm_raise_lower_speed)
    
    print("Arm raised")
    
    for _ in range(2):
        print("Waving!")
        
        move_servos([SERVO2_INDEX],
                    [elbow_flexation_1],
                    [elbow_flexation_2],
                    wave_speed)
        
        move_servos([SERVO2_INDEX],
                    [elbow_flexation_2],
                    [elbow_flexation_1],
                    wave_speed)
    
    print("Lowering arms!")
    
    move_servos([SERVO1_INDEX, SERVO2_INDEX, SERVO3_INDEX],
                [shoulder_flexation, elbow_flexation_1, gripper_open],
                [SERVO1_NEUTRAL, SERVO2_NEUTRAL, SERVO3_NEUTRAL],
                arm_raise_lower_speed)
    
    print("Done waving!")
    
    #disable servos used in this animation
    disable_channels([SERVO1_INDEX, SERVO2_INDEX, SERVO3_INDEX])
    

        
#raises arms to crab position and opens and gloses the grippers
def crab():
    global servos
    
    #speeds
    arm_raise_lower_speed = 0.02
    gripper_open_close = 0.01
    
    #number of times he opens and closes his claws
    num_pinches = 4
    
    #animation keyframe degree values
    right_shoulder_flexation = 66
    right_elbow_flexation = 128
    right_gripper_open = 102
    left_shoulder_flexation = 138
    left_elbow_flexation = 43
    left_gripper_open = 160
    
    print("Raising arms")
    
    #use new fancy function to move servos IN UNISON
    move_servos([SERVO1_INDEX, SERVO2_INDEX, SERVO5_INDEX, SERVO6_INDEX],
                [SERVO1_NEUTRAL, SERVO2_NEUTRAL, SERVO5_NEUTRAL, SERVO6_NEUTRAL],
                [right_shoulder_flexation, right_elbow_flexation, left_shoulder_flexation, left_elbow_flexation],
                arm_raise_lower_speed)
    
    print("Beginning crab")
    
    for _ in range(num_pinches):
        #open grippers
        move_servos([SERVO3_INDEX, SERVO7_INDEX],
                    [SERVO3_NEUTRAL, SERVO7_NEUTRAL],
                    [right_gripper_open, left_gripper_open],
                    gripper_open_close)
        
        #close grippers
        move_servos([SERVO3_INDEX, SERVO7_INDEX],
                    [right_gripper_open, left_gripper_open],
                    [SERVO3_NEUTRAL, SERVO7_NEUTRAL],
                    gripper_open_close)
        
        print("CRAB")
    
    print("lowering arms")
    move_servos([SERVO1_INDEX, SERVO2_INDEX, SERVO5_INDEX, SERVO6_INDEX],
                [right_shoulder_flexation, right_elbow_flexation, left_shoulder_flexation, left_elbow_flexation],
                [SERVO1_NEUTRAL, SERVO2_NEUTRAL, SERVO5_NEUTRAL, SERVO6_NEUTRAL],
                arm_raise_lower_speed)
    
    print("done crabbing!")
    
    #disable the channels we just used
    disable_channels([SERVO1_INDEX, SERVO2_INDEX, SERVO3_INDEX, SERVO5_INDEX, SERVO6_INDEX, SERVO7_INDEX])

#reaches right arm forward and opens gripper to receive gift
def reach_right_arm():
    global servos
    
    #animation keyframe degree positions
    shoulder_rotation = 66 #rotate shoulder so arm can reach forwards
    shoulder_flexation = 107 #raise arm a little
    elbow_flexation = 78 #raise hand a little
    gripper_open = 102 #open position for gripper
    
    #speed variables
    arm_raise_lower_speed = 0.020 #20 milliseconds
    gripper_open_close_speed = 0.010 #10 milliseconds
    
    print("Raising arm")
    
    move_servos([SERVO0_INDEX, SERVO1_INDEX, SERVO2_INDEX],
                [SERVO0_NEUTRAL, SERVO1_NEUTRAL, SERVO2_NEUTRAL],
                [shoulder_rotation, shoulder_flexation, elbow_flexation],
                arm_raise_lower_speed)
    
    print("Arm raised")
    
    print("Opening gripper")
    
    move_servos([SERVO3_INDEX],
                [SERVO3_NEUTRAL],
                [gripper_open],
                gripper_open_close_speed)
    
    print("Give me a treat!")
    
    #the next function called after this MUST BE retract_right_arm()

#picks up where reach_right_arm left off 
def retract_right_arm():
    global servos
    
    #assumes arms start in raised position
    #animation keyframe degree positions
    shoulder_rotation = 66 #rotate shoulder so arm can reach forwards
    shoulder_flexation = 107 #raise arm a little
    elbow_flexation = 78 #raise hand a little
    gripper_open = 102 #open position for gripper
    
    #speed variables
    arm_raise_lower_speed = 0.020 #20 milliseconds
    gripper_open_close_speed = 0.010 #10 milliseconds
    pause = 1.5 #1.5 seconds
    
    print("Closing gripper")
    
    move_servos([SERVO3_INDEX],
                [gripper_open],
                [SERVO3_NEUTRAL],
                gripper_open_close_speed)
    
    print("Pausing")
    
    #pause
    time.sleep(pause)
    
    print("Lowering arm")
    
    move_servos([SERVO0_INDEX, SERVO1_INDEX, SERVO2_INDEX],
                [shoulder_rotation, shoulder_flexation, elbow_flexation],
                [SERVO0_NEUTRAL, SERVO1_NEUTRAL, SERVO2_NEUTRAL],
                arm_raise_lower_speed)
    
    print("Arm lowered")
    
    #disable servos
    disable_channels([SERVO0_INDEX, SERVO1_INDEX, SERVO2_INDEX, SERVO3_INDEX])
    
#drillbit raises both arms, one holding diploma
#then he releases the diploma while mimicing a handshake
#then he lowers his arms
def give_diploma():
    global servos
    
    #animation keyframe angles
    right_shoulder_rotation = 66
    right_shoulder_flexation = 107
    right_elbow_flexation_1 = 78
    right_elbow_flexation_2 = 108
    left_shoulder_rotation = 131
    left_shoulder_flexation = 98
    left_elbow_flexation = 93
    left_gripper_open = 160
    
    #speeds
    arm_raise_lower_speed = 0.02
    handshake_speed = 0.01
    gripper_open_close_speed = 0.01
    pause = 2
    
    print("Raising arms")
    
    move_servos([SERVO0_INDEX, SERVO1_INDEX, SERVO2_INDEX, SERVO4_INDEX, SERVO5_INDEX, SERVO6_INDEX],
                [SERVO0_NEUTRAL, SERVO1_NEUTRAL, SERVO2_NEUTRAL, SERVO4_NEUTRAL, SERVO5_NEUTRAL, SERVO6_NEUTRAL],
                [right_shoulder_rotation, right_shoulder_flexation, right_elbow_flexation_1, left_shoulder_rotation, left_shoulder_flexation, left_elbow_flexation],
                arm_raise_lower_speed)
    
    print("Shaking hands!")
    
    for _ in range(2):
        #shake hand up
        move_servos([SERVO2_INDEX],
                    [right_elbow_flexation_1],
                    [right_elbow_flexation_2],
                    handshake_speed)
        
        #shake hand down
        move_servos([SERVO2_INDEX],
                    [right_elbow_flexation_2],
                    [right_elbow_flexation_1],
                    handshake_speed)
        
    print("Giving diploma!")
    
    move_servos([SERVO7_INDEX],
                [SERVO7_NEUTRAL],
                [left_gripper_open],
                gripper_open_close_speed)
    
    print("Pausing for picture")
    
    time.sleep(pause)
    
    print("Lowering arms")
    
    move_servos([SERVO0_INDEX, SERVO1_INDEX, SERVO2_INDEX, SERVO4_INDEX, SERVO5_INDEX, SERVO6_INDEX, SERVO7_INDEX],
                [right_shoulder_rotation, right_shoulder_flexation, right_elbow_flexation_1, left_shoulder_rotation, left_shoulder_flexation, left_elbow_flexation, left_gripper_open],
                [SERVO0_NEUTRAL, SERVO1_NEUTRAL, SERVO2_NEUTRAL, SERVO4_NEUTRAL, SERVO5_NEUTRAL, SERVO6_NEUTRAL, SERVO7_NEUTRAL],
                arm_raise_lower_speed)
    
    print("Done! Congratulations on graduating from UCF!")
    
#raises arms a little so his grippers don't hit the base
def raise_arms_for_rotation(): 
    #animation variables
    right_elbow_flexation = 58
    left_elbow_flexation = 113
    
    #speed
    arm_speed = 0.001 #1 ms delay, very fast!
    
    #raise arms
    move_servos([SERVO2_INDEX, SERVO6_INDEX],
                [SERVO2_NEUTRAL, SERVO6_NEUTRAL],
                [right_elbow_flexation, left_elbow_flexation],
                arm_speed)
    
    print("Arms raised for rotation")
    
#puts arms back to neutral after rotation and disables channels
def lower_arms_after_rotation(): 
    #animation variables
    right_elbow_flexation = 58
    left_elbow_flexation = 113
    
    #speed
    arm_speed = 0.001 #1 ms delay, very fast!
    
    #raise arms
    move_servos([SERVO2_INDEX, SERVO6_INDEX],
                [right_elbow_flexation, left_elbow_flexation],
                [SERVO2_NEUTRAL, SERVO6_NEUTRAL],
                arm_speed)
    
    print("Arms lowered after rotation")
    
    #disable servos
    disable_channels([SERVO2_INDEX, SERVO6_INDEX])
    
#nods head up and down in a 'yes' motion
def nod_head():
    global servo9_current_pos
    
    #animation variable
    head_nod_up = 140
    num_nods = 2
    
    #speed
    head_nod_speed = 0.005 #5 milliseconds
    
    #ensure head is a neutral position
    move_servos([SERVO9_INDEX], [servo9_current_pos], [SERVO9_NEUTRAL], head_nod_speed)
    
    for _ in range(num_nods):
        print("Nodding!")
        
        #move servo to up position
        move_servos([SERVO9_INDEX],
                    [SERVO9_NEUTRAL],
                    [head_nod_up],
                    head_nod_speed)
        
        #move servo to down position
        move_servos([SERVO9_INDEX],
                    [head_nod_up],
                    [SERVO9_NEUTRAL],
                    head_nod_speed)
        
    #disable servo channel
    disable_channels([SERVO9_INDEX])
    
    #update global variable
    servo9_current_pos = SERVO9_NEUTRAL

#tilts head to the 
def tilt_head_confused():
    global servo8_current_pos
    print("Tilting head to confused")
    
    #animation keyframe
    head_tilt = 20 #tilt head right
    
    #speed
    head_tilt_speed = 0.005 # 5 milliseconds
    
    #tilt head
    move_servos([SERVO8_INDEX],
                [servo8_current_pos],
                [head_tilt],
                head_tilt_speed)
    
    #update global variables
    servo8_current_pos = head_tilt

#tilts head back to neutral position and disables the servo channel
def head_tilt_neutral():
    global servo8_current_pos
    global servo9_current_pos
    
    print("Tilting head back to neutral")
    
    #speed
    head_tilt_speed = 0.005 # 5 milliseconds
    
    #tilt head
    move_servos([SERVO8_INDEX],
                [servo8_current_pos],
                [SERVO8_NEUTRAL],
                head_tilt_speed)
    
    move_servos([SERVO9_INDEX],
                [servo9_current_pos],
                [SERVO9_NEUTRAL],
                head_tilt_speed)
    
    #update global variables
    servo8_current_pos = SERVO8_NEUTRAL
    servo9_current_pos = SERVO9_NEUTRAL
    
    #diable channel
    disable_channels([SERVO8_INDEX, SERVO9_INDEX])
    
# **** END SERVO FUNCTIONS

# **** ANIMATION FUNCTIONS ****

def neutral():
    print("Neutral")
    neutral_eyes()
    neutral_eyebrows()
    head_tilt_neutral()

def happy():
    print("Happy")
    happy_eyes()
    neutral_eyebrows()
    
def sad():
    print("Sad")
    sad_eyes()
    sad_eyebrows()
    
def confused():
    print("Confused")
    shocked_eyes()
    confused_eyebrows()
    tilt_head_confused()

def angry():
    print("Angry")
    angry_eyes()
    angry_eyebrows()
    crab()

def yes():
    print("Yes")
    happy_eyes()
    nod_head()
    neutral_eyes()
    
def greeting():
    print("Greeting")
    happy_eyes()
    wave_right_arm()
    neutral_eyes()

#reaches forwards to recieve a gift
def reach_forward():
    print("Reaching forward")
    reach_right_arm()

#robot is happy after receiving a gift
def reach_backward():
    print("Reaching backwards")
    retract_right_arm()
    happy_eyes()

#robot is happy to see us graduate
def diploma():
    happy_eyes()
    give_diploma()
    neutral_eyes
    
    
#LED FUNCTIONS

#function to turn off LED screens
def all_LEDs_off():
    global pixels
    
    pixels.fill((0, 0, 0))
    
    pixels.show()

#displays yellow, circular eyes with pupils
def neutral_eyes():
    #define yellow color
    r = 255
    g = 255
    b = 0
    
    #define which LEDs should be lit or not
    neutral_list = [
          (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0),
          (0,0,0), (0,0,0), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (0,0,0), (0,0,0),
          (0,0,0), (r,g,b), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (r,g,b), (0,0,0),
          (0,0,0), (r,g,b), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (r,g,b), (0,0,0),
          (0,0,0), (r,g,b), (r,g,b), (r,g,b), (0,0,0), (0,0,0), (r,g,b), (0,0,0),
          (0,0,0), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (0,0,0), (r,g,b), (0,0,0),
          (0,0,0), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (0,0,0), (r,g,b), (0,0,0),
          (0,0,0), (0,0,0), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (0,0,0), (0,0,0),
          (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0),
          (0,0,0), (0,0,0), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (0,0,0), (0,0,0),
          (0,0,0), (r,g,b), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (r,g,b), (0,0,0),
          (0,0,0), (r,g,b), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (r,g,b), (0,0,0),
          (0,0,0), (r,g,b), (0,0,0), (0,0,0), (r,g,b), (r,g,b), (r,g,b), (0,0,0),
          (0,0,0), (r,g,b), (0,0,0), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (0,0,0),
          (0,0,0), (r,g,b), (0,0,0), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (0,0,0),
          (0,0,0), (0,0,0), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (0,0,0), (0,0,0)
        ]
    
    #turn all the pixels off before switching to the new eyes
    all_LEDs_off()
    
    #write correct color to each LED
    for i in range(0, 127):
        pixels[i] = neutral_list[i]
        
    pixels.show()

#displays green, half moon happy eyes
def happy_eyes():
    #define green color
    r = 0
    g = 255
    b = 0
    
    #define which LEDS should be lit or not
    super_happy_list = [
        (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0),
        (0,0,0), (0,0,0), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (0,0,0), (0,0,0),
        (0,0,0), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (0,0,0),
        (0,0,0), (r,g,b), (r,g,b), (0,0,0), (0,0,0), (r,g,b), (r,g,b), (0,0,0),
        (0,0,0), (r,g,b), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (r,g,b), (0,0,0),
        (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0),
        (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0),
        (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0),
        (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0),
        (0,0,0), (0,0,0), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (0,0,0), (0,0,0),
        (0,0,0), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (0,0,0),
        (0,0,0), (r,g,b), (r,g,b), (0,0,0), (0,0,0), (r,g,b), (r,g,b), (0,0,0),
        (0,0,0), (r,g,b), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (r,g,b), (0,0,0),
        (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0),
        (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0),
        (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0),
        ]
    
    #turn all the pixels off before switching to the new eyes
    all_LEDs_off()
    
    #write vlaue to each LED
    for i in range(0, 127):
        pixels[i] = super_happy_list[i]
        
    pixels.show()

#disaplys blue, downturned sad eyes
def sad_eyes():
    #define dark purple color
    r = 0
    g = 51
    b = 204
    
    #define which LEDs should be lit and which should not
    sad_list = [
            (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0),
            (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0),
            (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0),
            (0,0,0), (r,g,b), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (r,g,b), (0,0,0),
            (0,0,0), (r,g,b), (r,g,b), (0,0,0), (0,0,0), (r,g,b), (r,g,b), (0,0,0),
            (0,0,0), (0,0,0), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (0,0,0), (0,0,0),
            (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0),
            (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0),
            (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0),
            (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0),
            (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0),
            (0,0,0), (r,g,b), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (r,g,b), (0,0,0),
            (0,0,0), (r,g,b), (r,g,b), (0,0,0), (0,0,0), (r,g,b), (r,g,b), (0,0,0),
            (0,0,0), (0,0,0), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (0,0,0), (0,0,0),
            (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0),
            (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0)
        ]
    
    #turn all the pixels off before switching to the new eyes
    all_LEDs_off()
    
    #write values to LED pixels
    for i in range(0, 127):
        pixels[i] = sad_list[i]
    
    pixels.show()

#displays white, wide shocked eyes
def shocked_eyes():
    #define white color
    r = 255
    g = 255
    b = 255
    
    #define which pixels should be lit or not
    shocked_list = [
            (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0),
            (0,0,0), (0,0,0), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (0,0,0), (0,0,0),
            (0,0,0), (r,g,b), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (r,g,b), (0,0,0),
            (0,0,0), (r,g,b), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (r,g,b), (0,0,0),
            (0,0,0), (r,g,b), (0,0,0), (r,g,b), (r,g,b), (0,0,0), (r,g,b), (0,0,0),
            (0,0,0), (r,g,b), (0,0,0), (r,g,b), (r,g,b), (0,0,0), (r,g,b), (0,0,0),
            (0,0,0), (r,g,b), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (r,g,b), (0,0,0),
            (0,0,0), (0,0,0), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (0,0,0), (0,0,0),
            (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0),
            (0,0,0), (0,0,0), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (0,0,0), (0,0,0),
            (0,0,0), (r,g,b), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (r,g,b), (0,0,0),
            (0,0,0), (r,g,b), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (r,g,b), (0,0,0),
            (0,0,0), (r,g,b), (0,0,0), (r,g,b), (r,g,b), (0,0,0), (r,g,b), (0,0,0),
            (0,0,0), (r,g,b), (0,0,0), (r,g,b), (r,g,b), (0,0,0), (r,g,b), (0,0,0),
            (0,0,0), (r,g,b), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (r,g,b), (0,0,0),
            (0,0,0), (0,0,0), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (0,0,0), (0,0,0)
        ]
    
    #turn all the pixels off before switching to the new eyes
    all_LEDs_off()
    
    #write values to pixels
    for i in range(0, 127):
        pixels[i] = shocked_list[i]
    
    pixels.show()

#displays red, angry eyes
def angry_eyes():
    #define red color
    r = 255
    g = 0
    b = 0
    
    #define which pixels should be lit and which should not
    angry_list = [
            (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0),
            (0,0,0), (0,0,0), (0,0,0), (0,0,0), (r,g,b), (r,g,b), (0,0,0), (0,0,0),
            (0,0,0), (0,0,0), (0,0,0), (r,g,b), (0,0,0), (0,0,0), (r,g,b), (0,0,0),
            (0,0,0), (0,0,0), (r,g,b), (0,0,0), (0,0,0), (0,0,0), (r,g,b), (0,0,0),
            (0,0,0), (r,g,b), (r,g,b), (r,g,b), (0,0,0), (0,0,0), (r,g,b), (0,0,0),
            (0,0,0), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (0,0,0), (r,g,b), (0,0,0),
            (0,0,0), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (0,0,0), (r,g,b), (0,0,0),
            (0,0,0), (0,0,0), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (0,0,0), (0,0,0),
            (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0), (0,0,0),
            (0,0,0), (0,0,0), (r,g,b), (r,g,b), (0,0,0), (0,0,0), (0,0,0), (0,0,0),
            (0,0,0), (r,g,b), (0,0,0), (0,0,0), (r,g,b), (0,0,0), (0,0,0), (0,0,0),
            (0,0,0), (r,g,b), (0,0,0), (0,0,0), (0,0,0), (r,g,b), (0,0,0), (0,0,0),
            (0,0,0), (r,g,b), (0,0,0), (0,0,0), (r,g,b), (r,g,b), (r,g,b), (0,0,0),
            (0,0,0), (r,g,b), (0,0,0), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (0,0,0),
            (0,0,0), (r,g,b), (0,0,0), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (0,0,0),
            (0,0,0), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (r,g,b), (0,0,0), (0,0,0)
        ]
    
    #turn all the pixels off before switching to the new eyes
    all_LEDs_off()
    
    #write values to pixels
    for i in range(0, 127):
        pixels[i] = angry_list[i]
    
    pixels.show()

# **** MOTOR FUNCTIONS ****

#looks at the global movement array and switches to correct drive function
def update_drive_movement():
    global drive_event_counter
    global drive
    
    #print(f"Drive movement counter: {drive_event_counter}")
    
    #check if we need to update current movement
    #this will happen when a counter hits its max value
    update = False
    for i in range(0, 2):
        if drive_event_counter[i] > drive_event_counter_max:
            #no movement detected in this direction for too long
            #check if this is new information
            if drive_current_movement[i] != False:
                #this is new, make update
                update = True
                drive_current_movement[i] = False
                
            #set counter back to max so numbers don't get too big
            drive_event_counter[i] = drive_event_counter_max
        elif drive_event_counter[i] < drive_event_counter_max:
            #check if this is new information
            if drive_current_movement[i] != True:
                #this is new, make update
                update = True
                drive_current_movement[i] = True
            
    #if we made an update, call corresponding function
    #drive_current_movement array is [Forward, Reverse, Left, Right]
    if update:
        if drive_current_movement == [False, False]:
            stop_drive_motors()
        elif drive_current_movement == [True, False]:
            forward()
        elif drive_current_movement == [False, True]:
            reverse()

def forward():
    print("Moving forwards!")
    
    #move left motor forwards
    GPIO.output(m1_in1_pin,GPIO.HIGH)
    GPIO.output(m1_in2_pin,GPIO.LOW)
    
    #move right motor forwards
    GPIO.output(m2_in1_pin,GPIO.HIGH)
    GPIO.output(m2_in2_pin,GPIO.LOW)
    
def reverse():
    print("Reversing!")
    
    #move left motor backwardss
    GPIO.output(m1_in1_pin,GPIO.LOW)
    GPIO.output(m1_in2_pin,GPIO.HIGH)
    
    #move right motor backwards
    GPIO.output(m2_in1_pin,GPIO.LOW)
    GPIO.output(m2_in2_pin,GPIO.HIGH)
    
def stop_drive_motors():
    #stop left motor
    GPIO.output(m1_in1_pin,GPIO.LOW)
    GPIO.output(m1_in2_pin,GPIO.LOW)
    
    #stop right motor
    GPIO.output(m2_in1_pin,GPIO.LOW)
    GPIO.output(m2_in2_pin,GPIO.LOW)
    
    print("Stopping drive motors")

#beging to rotate torso
def begin_torso_rotation(going_right):
    #raise arms a little
    raise_arms_for_rotation()
    
    #set direction pins correctly
    if going_right:
        print("Moving to the right.")
        GPIO.output(m3_in1_pin,GPIO.HIGH)
        GPIO.output(m3_in2_pin,GPIO.LOW)
    else:
        print("Moving to the left.")
        GPIO.output(m3_in1_pin,GPIO.LOW)
        GPIO.output(m3_in2_pin,GPIO.HIGH)

#ends torso rotation
def end_torso_rotation():
    #stop motor
    GPIO.output(m3_in1_pin,GPIO.LOW)
    GPIO.output(m3_in2_pin,GPIO.LOW)
    
    print("Torso rotation ended")
    
    lower_arms_after_rotation()
    

# **** MAIN FUNCTIONS ****

#ends program execution by changing RUNNING variable used in main function
def stop():
    global RUNNING #change global variable
    print("Stopping!")
    RUNNING = False
    
    #turn off LEDs
    all_LEDs_off()
    
    #turn off all motors
    end_torso_rotation()
    stop_drive_motors()
    
    #disable all servo channels
    disable_channels([SERVO0_INDEX, SERVO1_INDEX, SERVO2_INDEX, SERVO3_INDEX,
                      SERVO4_INDEX, SERVO5_INDEX, SERVO6_INDEX, SERVO7_INDEX])
    
    #cleanup GPIO
    GPIO.cleanup()


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
    #print("Performance mode loop!")
    #check if new message received from Raspberry Pi
    try:
        raw_msg = connection.recv(1024)
    except socket.timeout:
        #socket timed out, no new message
        #return back to main function for next loop
        pass
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
        elif msg == "g":
            greeting()
        elif msg == "rf":
            reach_forward()
        elif msg == "rb":
            reach_backward()
        elif msg == "d":
            diploma()
        elif msg == "n":
            neutral()
        elif msg == "q":
            move_to_neutral()
        else:
            connection.send(bytes("I don't know what you want from me!", "utf-8"))
            
    #check for message from Driver to switch to drive mode
    for event in joycon.read_loop():
        #print("checking joycons")
        #filter by event type
        if event.type == ecodes.EV_KEY:
            #check which button was pressed
            if(event.code == 305):
                #X button pressed, stop!!
                stop()
            elif(event.code == 308 and event.value == 1):
                #B button pressed, switch to drive mode
                switch_mode(performance=False)
        break


#reads input coming from JoyCon
def drive_mode_loop():
    print("Drive mode loop!")
    t0 = time.time()
    #check for input from JoyCon
    #use evdec function to continuously read joycon inputs
    for event in joycon.read_loop():
        #filter by event type
        if event.type == ecodes.EV_KEY:
            #check which button was pressed
            if(event.code == 304 and event.value == 1):
                #A button pressed, switch to performance mode
                switch_mode(performance=True)
                break
            elif(event.code == 305 and event.value ==1):
                #X button pressed, stop!!
                stop()
                break
        elif event.type == ecodes.EV_ABS:
            #filter joystick events
            #filter by left/right or up/down
            if(event.code == UPDOWN_CODE):
                #filter by up or down and value
                if(event.value < JOYCON_FORWARD_CUTOFF):
                    drive_event_counter[COUNTER_F_INDEX] = 0
                    
                    #swiftly disable backward movement
                    drive_event_counter[COUNTER_B_INDEX] = drive_event_counter_max + 1
                elif(event.value > JOYCON_BACKWARD_CUTOFF):
                    drive_event_counter[COUNTER_B_INDEX] = 0
                    
                    #swiftly disable forward movement
                    drive_event_counter[COUNTER_F_INDEX] = drive_event_counter_max + 1
                    
        #update time
        dt = time.time()-t0
        t0 = time.time()
        for i in range(0, 2):
            drive_event_counter[i] += dt
                    
        update_drive_movement()
    
#     #check for Performance Controller command to switch back to performance mode
#     #check if new message received from Raspberry Pi
#     try:
#         raw_msg = connection.recv(1024)
#     except socket.timeout:
#         #socket timed out, no new message
#         #return back to main function for next loop
#         return
#     else:
#         #message found, process message
#         msg = raw_msg.decode("utf-8")
#         
#         #enter performance mode if that was the command
#         if msg == "p":
#             switch_mode(performance=True)
        
        
def main():
    #start by setting the eyes to neutral
    all_LEDs_off()
    neutral_eyes()
    
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
servos = ServoKit(channels=16)

initialize_servos()

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
m1_speed_pwm.start(100) #default speed is 100% of duty cycle

print("Motor 1 initialized.")

m2_in1_pin = 23 #forward
m2_in2_pin = 24 #backward
m2_speed_pin = 25
GPIO.setup(m2_in1_pin,GPIO.OUT)
GPIO.setup(m2_in2_pin,GPIO.OUT)
GPIO.setup(m2_speed_pin,GPIO.OUT)
GPIO.output(m2_in1_pin,GPIO.LOW)
GPIO.output(m2_in2_pin,GPIO.LOW)
m2_speed_pwm = GPIO.PWM(m2_speed_pin,1000)
m2_speed_pwm.start(100) #default speed is 100% of duty cycle

print("Motor 2 initialized.")

#Second L298N motor driver
#Connected to motor 3 (torso rotation motor)
m3_in1_pin = 5 #high = right
m3_in2_pin = 6 #high = left
m3_speed_pin = 26
GPIO.setup(m3_in1_pin,GPIO.OUT)
GPIO.setup(m3_in2_pin,GPIO.OUT)
GPIO.setup(m3_speed_pin,GPIO.OUT)
GPIO.output(m3_in1_pin,GPIO.LOW)
GPIO.output(m3_in2_pin,GPIO.LOW)
m3_speed_pwm = GPIO.PWM(m3_speed_pin,1000)
m3_current_speed = 85 #default speed is 100% of duty cycle
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
    pixel_pin, num_pixels, brightness=0.05, auto_write=False, pixel_order=ORDER
)

print("LED screens initialized.")

print("Finished initializing GPIO connections!")


# **** BLUETOOTH SETUP ****
print("Beginning setup of Bluetooth device.")

#create object to store joycon data
joycon = InputDevice(JOYCON_PATH)

#joystick movement tracking
#number of second where movement IS NOT detected before shutting off
drive_event_counter_max = 0.5
#tracks consecutive seconds where [forward, backward] is NOT detected
drive_event_counter = [drive_event_counter_max, drive_event_counter_max]
COUNTER_F_INDEX = 0
COUNTER_B_INDEX = 1
drive_current_movement = [False, False] #[forward, backward]
#unsigned integers to compare with evdev objects variables
UPDOWN_CODE = 2

#cutoff points for what counts as a joystick event
JOYCON_FORWARD_CUTOFF = -900
JOYCON_BACKWARD_CUTOFF = 900

#print out info at start
print(joycon)

print("Bluetooth setup complete!")


# **** SOCKET SETUP ****
print("Beginning socket setup.")

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

print("Setup complete! Ready to receive commands!")

# **** END SETUP ****

#start listening for input
main()

#close socket connection when program ends
connection.close()

print("Connection closed, thank you!")

