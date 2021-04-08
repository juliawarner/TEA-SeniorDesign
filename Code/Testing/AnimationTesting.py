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
SERVO7_NEUTRAL = 179 #down
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
#waves right arm
def wave_right_arm():
    global servos
    
    print("Starting wave! Hello!")
    
    #animation keyframe degrees
    shoulder_flexation = 135 #arm rasied to wave
    elbow_flexation_1 = 118 #starting point for wave
    elbow_flexation_2 = 138 #ending point for wave
    gripper_open = 102 #open position for gripper

    #animation speeds, delay in seconds between each change in servo position
    arm_raise_lower_speed = 0.015 #15 milliseconds
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
        
        move_servos([SERVO0_INDEX],
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
    
    #pause
    time.sleep(pause)
    
    move_servos([SERVO0_INDEX, SERVO1_INDEX, SERVO2_INDEX],
                [shoulder_rotation, shoulder_flexation, elbow_flexation],
                [SERVO0_NEUTRAL, SERVO1_NEUTRAL, SERVO2_NEUTRAL],
                arm_raise_lower_speed)
    
    print("Arm lowered")
    
    #disable servos
    disable_channels([SERVO0_INDEX, SERVO1_INDEX, SERVO2_INDEX, SERVO3_INDEX])

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

#raises arms to crab position and opens and gloses the grippers
def new_crab():
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
#     single_servo_test(SERVO0_INDEX, 156, 136, 0.02)
#     single_servo_test(SERVO1_INDEX, 157, 137, 0.02)
#     single_servo_test(SERVO2_INDEX, 38, 58, 0.02)
#     single_servo_test(SERVO3_INDEX, 82, 102, 0.02)
#     single_servo_test(SERVO4_INDEX, 41, 61, 0.02)
#     single_servo_test(SERVO5_INDEX, 48, 68, 0.02)
#     single_servo_test(SERVO6_INDEX, 133, 113, 0.02)
    single_servo_test(SERVO7_INDEX, 179, 160, 0.02)
    
# function init 
def init():
    global servos

    for i in range(16):
        servos.servo[i].set_pulse_width_range(MIN_IMP[i] , MAX_IMP[i])
        servos.servo[i].actuation_range = 180

init()


#move_to_neutral()

#move_servos([0], [180], [0], 0.01)

#new_crab()

#test_all_servos()
#wave_right_arm()
reach_right_arm()
retract_right_arm()



