import time
from adafruit_servokit import ServoKit

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
SERVO7_NEUTRAL = 180 #down
SERVO7_INDEX = 7

#left eyebrow
SERVO8_NEUTRAL = 0
SERVO8_INDEX = 8

#right eyebrow
SERVO9_NEUTRAL = 0
SERVO9_INDEX = 9


#Parameters for max/min PWM impulses
MIN_IMP  =[500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500]
MAX_IMP  =[2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500, 2500]

# Set channels to the number of servo channels on your kit.
servos = ServoKit(channels=16)

#servos.servo[0].angle = 180
#time.sleep(2)
#servos.servo[0].angle = 0
#time.sleep(2)

#function to test animation for waving the bot's right arm
#assumes:
#        right shoulder rotation = servos.servo[0]
#        right shoulder flaxation = servos.servo[1]
#        right elbow flexation = servos.servo[2]
#        right gripper = servos.servo[3]
#        all servos start from neutral position
#changes servos object global variable
def wave_right_arm():
    #specify we will be editing global variable
    global servos

    print("Starting wave! Hello!")
    
    #start at neutral
    servos.servo[SERVO1_INDEX].angle = SERVO1_NEUTRAL
    servos.servo[SERVO2_INDEX].angle = SERVO2_NEUTRAL
    servos.servo[SERVO3_INDEX].angle = SERVO3_NEUTRAL

    #animation keyframe degrees
    shoulder_flexation = 100 #arm rasied to wave
    elbow_flexation_1 = 68 #starting point for wave
    elbow_flexation_2 = 98 #ending point for wave
    gripper_open = 102 #open position for gripper

    #animation speeds, delay in seconds between each change in servo position
    arm_rase_lower_speed = 0.020 #20 milliseconds
    wave_speed = 0.010 #10 milliseconds

    #raise arm for wave and open gripper
    while(servos.servo[SERVO1_INDEX].angle > shoulder_flexation or servos.servo[SERVO2_INDEX].angle < elbow_flexation_1):
        #add or subtract but make sure target isn't overshot
        servos.servo[SERVO1_INDEX].angle = servos.servo[SERVO1_INDEX].angle - 1 if servos.servo[SERVO1_INDEX].angle > shoulder_flexation else shoulder_flexation
        servos.servo[SERVO2_INDEX].angle = servos.servo[SERVO2_INDEX].angle + 1 if servos.servo[SERVO2_INDEX].angle < elbow_flexation_1 else elbow_flexation_1
        servos.servo[SERVO3_INDEX].angle = servos.servo[SERVO3_INDEX].angle + 1 if servos.servo[SERVO3_INDEX].angle < gripper_open else gripper_open

        time.sleep(arm_rase_lower_speed)

    print("Arm raised!")

    #wave back and forth twice
    for i in range(2):
        print("Waving!")
        #move between two elbow flexation points
        while(servos.servo[SERVO2_INDEX].angle < elbow_flexation_2):
            servos.servo[SERVO2_INDEX].angle += 1

            time.sleep(wave_speed)

        while(servos.servo[SERVO2_INDEX].angle > elbow_flexation_1):
            servos.servo[SERVO2_INDEX].angle -= 1

            time.sleep(wave_speed)

    print("Lowering arm!")

    #lower arm after wave and close gripper
    while(servos.servo[SERVO1_INDEX].angle < SERVO1_NEUTRAL or servos.servo[SERVO2_INDEX].angle > SERVO2_NEUTRAL):
        #add or subtract but make sure target isn't overshot
        servos.servo[SERVO1_INDEX].angle = servos.servo[SERVO1_INDEX].angle + 1 if servos.servo[SERVO1_INDEX].angle < SERVO1_NEUTRAL else SERVO1_NEUTRAL
        servos.servo[SERVO2_INDEX].angle = servos.servo[SERVO2_INDEX].angle - 1 if servos.servo[SERVO2_INDEX].angle > SERVO2_NEUTRAL else SERVO2_NEUTRAL
        servos.servo[SERVO3_INDEX].angle = servos.servo[SERVO3_INDEX].angle - 1 if servos.servo[SERVO3_INDEX].angle > SERVO3_NEUTRAL else SERVO3_NEUTRAL

        time.sleep(arm_rase_lower_speed)
        
    #disable channels used in this animation
    servos.servo[SERVO1_INDEX].angle = None
    servos.servo[SERVO2_INDEX].angle = None
    servos.servo[SERVO3_INDEX].angle = None

    print("Done waving! Goodbye!")
    
#moves the right arm forwards, opening the gripper to receive a gift
def reach_right_arm():
    #specify we will be editing global variable
    global servos
    
    #start at neutral
    servos.servo[SERVO0_INDEX].angle = SERVO0_NEUTRAL
    servos.servo[SERVO1_INDEX].angle = SERVO1_NEUTRAL
    servos.servo[SERVO2_INDEX].angle = SERVO2_NEUTRAL
    servos.servo[SERVO3_INDEX].angle = SERVO3_NEUTRAL
    
    #animation keyframe degree positions
    shoulder_rotation = 66 #rotate shoulder so arm can reach forwards
    shoulder_flexation = 107 #raise arm a little
    elbow_flexation = 78 #raise hand a little
    gripper_open = 102 #open position for gripper
    
    
    #speed variables
    arm_raise_lower_speed = 0.020 #20 milliseconds
    gripper_open_close_speed = 0.010 #10 milliseconds
    
    #raise arm
    while(servos.servo[SERVO0_INDEX].angle > shoulder_rotation or servos.servo[SERVO1_INDEX].angle > shoulder_flexation or servos.servo[SERVO2_INDEX].angle < elbox_flexation):
        servos.servo[SERVO0_INDEX].angle = servos.servo[SERVO0INDEX].angle - 1 if servos.servo[SERVO0_INDEX].angle > shoulder_rotation else shoulder_rotation
        servos.servo[SERVO1_INDEX].angle = servos.servo[SERVO1INDEX].angle - 1 if servos.servo[SERVO1_INDEX].angle > shoulder_flexation else shoulder_flexation
        servos.servo[SERVO2_INDEX].angle = servos.servo[SERVO2INDEX].angle + 1 if servos.servo[SERVO2_INDEX].angle < elbow_flexation else elbow_flexation
        
        time.sleep(arm_raise_lower_speed)
        
    print("Arm raised!")
    
    while(servos.servo[SERVO3_INDEX].angle < gripper_open):
        servos.servo[SERVO3_INDEX].angle += 1
        
        time.sleep(gripper_open_close_speed)
    
    print("Gripper open!")
        
    #please call retract_arm() immediately after calling this function

#picks up where raise_arm leaves off
#closes the gripper and returns arm to neutral position
def retract_arm():
    global servos
    
    #assumes arms start in raised position
    
    #speed variables
    arm_raise_lower_speed = 0.020 #20 milliseconds
    gripper_open_close_speed = 0.010 #10 milliseconds
    
    #close gripper
    while(servos.servo[SERVO3_INDEX].angle > SERVO3_NEUTRAL):
        servos.servo[SERVO3_INDEX].angle -= 1
        
        time.sleep(gripper_open_close_speed)
    
    print("Gripper closed.")
    
    #raise arm
    while(servos.servo[SERVO0_INDEX].angle < SERVO0_NEUTRAL or servos.servo[SERVO1_INDEX].angle < SERVO1_NEUTRAL or servos.servo[SERVO2_INDEX].angle > SERVO2_NEUTRAL):
        servos.servo[SERVO0_INDEX].angle = servos.servo[SERVO0INDEX].angle + 1 if servos.servo[SERVO0_INDEX].angle < SERVO0_NEUTRAL else SERVO0_NEUTRAL
        servos.servo[SERVO1_INDEX].angle = servos.servo[SERVO1INDEX].angle + 1 if servos.servo[SERVO1_INDEX].angle < SERVO1_NEUTRAL else SERVO1_NEUTRAL
        servos.servo[SERVO2_INDEX].angle = servos.servo[SERVO2INDEX].angle - 1 if servos.servo[SERVO2_INDEX].angle > SERVO2_NEUTRAL else SERVO2_NEUTRAL
        
        time.sleep(arm_raise_lower_speed)
        
    print("Arm lowered")
        
    #disable channels used in this animation
    servos.servo[SERVO0_INDEX].angle = None
    servos.servo[SERVO1_INDEX].angle = None
    servos.servo[SERVO2_INDEX].angle = None
    servos.servo[SERVO3_INDEX].angle = None

#lifts arm up like he's gonna scratch his chin
def curious_arm():
    #specify we will be editing global variable
    global servos
    
    #start at neutral
    servos.servo[SERVO1_INDEX].angle = SERVO1_NEUTRAL
    servos.servo[SERVO2_INDEX].angle = SERVO2_NEUTRAL
    
    #animation angles
    shoulder_flexation = 57 #raise arm a lot
    elbow_flexation = 138 #raise hand a lot
    
    #speed variables
    arm_raise_lower_speed = 0.020 #20 milliseconds\
    pause = 2 #2 seconds
    
    print("Raising arm")
    
    while(servos.servo[SERVO1_INDEX].angle > shoulder_flexation or servos.servo[SERVO2_INDEX].angle < elbow_flexation):
        servos.servo[SERVO1_INDEX].angle = servos.servo[SERVO1_INDEX].angle - 1 if servos.servo[SERVO1_INDEX].angle > shoulder_flexation else shoulder_flexation
        servos.servo[SERVO2_INDEX].angle = servos.servo[SERVO2_INDEX].angle + 1 if servos.servo[SERVO2_INDEX].angle < elbow_flexation else elbow_flexation
    
        time.sleep(arm_raise_lower_speed)
    
    print("Pausing")
    
    time.sleep(pause)
    
    while(servos.servo[SERVO1_INDEX].angle < SERVO1_NEUTRAL or servos.servo[SERVO2_INDEX].angle > SERVO2_NEUTRAL):
        servos.servo[SERVO1_INDEX].angle = servos.servo[SERVO1_INDEX].angle + 1 if servos.servo[SERVO1_INDEX].angle < SERVO1_NEUTRAL else SERVO1_NEUTRAL
        servos.servo[SERVO2_INDEX].angle = servos.servo[SERVO2_INDEX].angle - 1 if servos.servo[SERVO2_INDEX].angle > SERVO2_NEUTRAL else SERVO2_NEUTRAL
        
        time.sleep(arm_raise_lower_speed)
        
    print("Arm lowered")
    
    #disable channels used in this animation
    servos.servo[SERVO1_INDEX].angle = None
    servos.servo[SERVO2_INDEX].angle = None
    
#does crab
def crab():
    global servos
    
    #start at neutral
    servos.servo[SERVO1_INDEX].angle = SERVO1_NEUTRAL
    servos.servo[SERVO2_INDEX].angle = SERVO2_NEUTRAL
    servos.servo[SERVO3_INDEX].angle = SERVO3_NEUTRAL
    servos.servo[SERVO5_INDEX].angle = SERVO5_NEUTRAL
    servos.servo[SERVO6_INDEX].angle = SERVO6_NEUTRAL
    servos.servo[SERVO7_INDEX].angle = SERVO7_NEUTRAL
    
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
    
    #assumes all servoes are changing the same amount
    while(servos.servo[SERVO1_INDEX].angle > right_shoulder_flexation):
        servos.servo[SERVO1_INDEX].angle -= 1
        servos.servo[SERVO2_INDEX].angle += 1
        servos.servo[SERVO5_INDEX].angle += 1
        servos.servo[SERVO6_INDEX].angle -= 1
        
        time.sleep(arm_raise_lower_speed)
        
    print("beginning crab")
    
    for i in range(num_pinches):
        #open grippers (assumes they are moving the same amount)
        while(servos.servo[SERVO3_INDEX].angle < right_gripper_open):
            servos.servo[SERVO3_INDEX].angle += 1
            servos.servo[SERVO7_INDEX].angle -= 1
            
            time.sleep(gripper_open_close)
        
        #close grippers
        while(servos.servo[SERVO3_INDEX].angle > SERVO3_NEUTRAL):
            servos.servo[SERVO3_INDEX].angle -= 1
            servos.servo[SERVO7_INDEX].angle += 1
            
        print("CRAB")
        
    
    print("Lowering arms")
    
    #assumes all servoes are changing the same amount
    while(servos.servo[SERVO1_INDEX].angle < SERVO1_NEUTRAL):
        servos.servo[SERVO1_INDEX].angle += 1
        servos.servo[SERVO2_INDEX].angle -= 1
        servos.servo[SERVO5_INDEX].angle -= 1
        servos.servo[SERVO6_INDEX].angle += 1
        
        time.sleep(arm_raise_lower_speed)
            
        
    print("Done crabbing")
    
    #disable channels used in this animation
    servos.servo[SERVO1_INDEX].angle = None
    servos.servo[SERVO2_INDEX].angle = None
    servos.servo[SERVO3_INDEX].angle = None
    servos.servo[SERVO5_INDEX].angle = None
    servos.servo[SERVO6_INDEX].angle = None
    servos.servo[SERVO7_INDEX].angle = None
        

#a small gesticulation animation
def gesticulate_1():
    global servos
    
    #start at neutral
    servos.servo[SERVO1_INDEX].angle = SERVO1_NEUTRAL
    servos.servo[SERVO5_INDEX].angle = SERVO5_NEUTRAL
    
    #animation angles
    left_shoulder_flexation = 78 #raise arm a little
    right_shoulder_flexation = 127 #raise arm a little
    
    #speed
    speed = 0.01
    pause = 1.5
    
    print("Raising arms")
    
    #raise arms a little bit
    while(servos.servo[SERVO1_INDEX].angle > right_shoulder_flexation or servos.servo[SERVO5_INDEX].angle < left_shoulder_flexation):
        servos.servo[SERVO1_INDEX].angle = servos.servo[SERVO1_INDEX].angle - 1 if servos.servo[SERVO1_INDEX].angle > right_shoulder_flexation else right_shoulder_flexation
        servos.servo[SERVO5_INDEX].angle = servos.servo[SERVO5_INDEX].angle + 1 if servos.servo[SERVO5_INDEX].angle < left_shoulder_flexation else left_shoulder_flexation
        
        time.sleep(speed)
    
    time.sleep(pause)
    
    print("Lowering arms")
    
    #lower arms a little bit
    while(servos.servo[SERVO1_INDEX].angle < SERVO1_NEUTRAL or servos.servo[SERVO5_INDEX].angle > SERVO5_NEUTRAL):
        servos.servo[SERVO1_INDEX].angle = servos.servo[SERVO1_INDEX].angle + 1 if servos.servo[SERVO1_INDEX].angle < SERVO1_NEUTRAL else SERVO1_NEUTRAL
        servos.servo[SERVO5_INDEX].angle = servos.servo[SERVO5_INDEX].angle - 1 if servos.servo[SERVO5_INDEX].angle > SERVO5_NEUTRAL else SERVO5_NEUTRAL
        
        time.sleep(speed)
        
    print("Done gesticulating")
    
    #disable channels used in this animation
    servos.servo[SERVO1_INDEX].angle = None
    servos.servo[SERVO5_INDEX].angle = None


#moves each servo one at a time to it's pre-determined neutral position
def move_to_neutral():
    #specify we will be editing global variable
    global servos

    #moves all servos to neutral position 1 at a time, with time in between to settle
    print("Setting servos to neutral position")

    servos.servo[SERVO0_INDEX].angle = SERVO0_NEUTRAL
    time.sleep(2)
    print("Servo 0 at neutral position")
    servos.servo[SERVO0_INDEX].angle = None #disable channel

    servos.servo[SERVO1_INDEX].angle = SERVO1_NEUTRAL
    time.sleep(2)
    print("Servo 1 at neutral position")
    servos.servo[SERVO1_INDEX].angle = None #disable channel

    servos.servo[SERVO2_INDEX].angle = SERVO2_NEUTRAL
    time.sleep(2)
    print("Servo 2 at neutral position")
    servos.servo[SERVO2_INDEX].angle = None #disable channel

    servos.servo[SERVO3_INDEX].angle = SERVO3_NEUTRAL
    time.sleep(2)
    print("Servo 3 at neutral position")
    servos.servo[SERVO3_INDEX].angle = None #disable channel
    
    servos.servo[SERVO4_INDEX].angle = SERVO4_NEUTRAL
    time.sleep(2)
    print("Servo 4 at neutral position")
    servos.servo[SERVO4_INDEX].angle = None #disable channel
    
    servos.servo[SERVO5_INDEX].angle = SERVO5_NEUTRAL
    time.sleep(2)
    print("Servo 5 at neutral position")
    servos.servo[SERVO5_INDEX].angle = None #disable channel
    
    servos.servo[SERVO6_INDEX].angle = SERVO6_NEUTRAL
    time.sleep(2)
    print("Servo 6 at neutral position")
    servos.servo[SERVO6_INDEX].angle = None #disable channel
    
    servos.servo[SERVO7_INDEX].angle = SERVO7_NEUTRAL
    time.sleep(2)
    print("Servo 7 at neutral position")
    servos.servo[SERVO7_INDEX].angle = None #disable channel

#moves the specified servo back and forth between the two specified angles 2 times
#speed is seconds delay between each 1 degree change in angle
def single_servo_test(servo_num, start_angle, end_angle, speed):
    global servos
    
    print(f"Moving servo {servo_num} from {start_angle} to {end_angle}!")
    
    servos.servo[servo_num].angle = start_angle
    
    
    for i in range(2):
        if(start_angle < end_angle):
            while servos.servo[servo_num].angle < end_angle:
                servos.servo[servo_num].angle += 1
                print(f"{servos.servo[servo_num].angle}")
                time.sleep(speed)
                
            time.sleep(0.5)
            
            while servos.servo[servo_num].angle > start_angle:
                servos.servo[servo_num].angle -= 1
                print(f"{servos.servo[servo_num].angle}")
                time.sleep(speed)
                
            time.sleep(0.5)
        else:
            while servos.servo[servo_num].angle > end_angle:
                servos.servo[servo_num].angle -= 1
                print(f"{servos.servo[servo_num].angle}")
                time.sleep(speed)
                
            time.sleep(0.5)
            
            while servos.servo[servo_num].angle < start_angle:
                servos.servo[servo_num].angle += 1
                print(f"{servos.servo[servo_num].angle}")
                time.sleep(speed)
            
            time.sleep(0.5)
            
    #disable channel
    servos.servo[servo_num].angle = None
    
    print(f"Done with servo {servo_num}!")

#moves each servo a little bit to make sure that it's working
def test_all_servos():
    single_servo_test(SERVO0_INDEX, 156, 136, 0.02)
    single_servo_test(SERVO1_INDEX, 157, 137, 0.02)
    single_servo_test(SERVO2_INDEX, 38, 58, 0.02)
    single_servo_test(SERVO3_INDEX, 82, 102, 0.02)
    single_servo_test(SERVO4_INDEX, 41, 61, 0.02)
    single_servo_test(SERVO5_INDEX, 48, 68, 0.02)
    single_servo_test(SERVO6_INDEX, 33, 53, 0.02)
    single_servo_test(SERVO7_INDEX, 180, 160, 0.02)

move_to_neutral()
test_all_servos()



