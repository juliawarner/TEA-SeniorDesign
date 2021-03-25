import time
from adafruit_servokit import ServoKit

#**** SERVO NEUTRAL POSITIONS ****
#resting position in degrees for each servo
#up/down indicates if increasing/decreasing the value keeps the servo within its range of motion
#right arm
SERVO0_NEUTRAL = 156 #down
SERVO1_NEUTRAL = 157 #down
SERVO2_NEUTRAL = 38 #up
SERVO3_NEUTRAL = 82 #up

#left arm
SERVO4_NEUTRAL = 41 #up
SERVO5_NEUTRAL = 48 #up
SERVO6_NEUTRAL = 33 #down
SERVO7_NEUTRAL = 180 #down

# Set channels to the number of servo channels on your kit.
servos = ServoKit(channels=16)

servos.servo[0].angle = 180
time.sleep(2)
servos.servo[0].angle = 0
time.sleep(2)

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

    #animation keyframe degrees
    shoulder_flexation = 100 #arm rasied to wave
    elbow_flexation_1 = 68 #starting point for wave
    elbow_flexation_2 = 98 #ending point for wave
    gripper_open = 102 #open position for gripper

    #animation speeds, delay in seconds between each change in servo position
    arm_rase_lower_speed = 0.020 #20 milliseconds
    wave_speed = 0.010 #10 milliseconds

    #raise arm for wave and open gripper
    while(servos.servo[1].angle > shoulder_flexation or servos.servo[2].angle < elbow_flexation_1):
        #add or subtract but make sure target isn't overshot
        servos.servo[1].angle = servos.servo[1].angle - 1 if servos.servo[1].angle > shoulder_flexation else shoulder_flexation
        servos.servo[2].angle = servos.servo[2].angle + 1 if servos.servo[2].angle < elbow_flexation_1 else elbow_flexation_1
        servos.servo[3].angle = servos.servo[3].angle + 1 if servos.servo[3].angle < gripper_open else gripper_open

        time.sleep(arm_rase_lower_speed)

    print("Arm raised!")

    #wave back and forth twice
    for i in range(2):
        print("Waving!")
        #move between two elbow flexation points
        while(servos.servo[2].angle < elbow_flexation_2):
            servos.servo[2].angle += 1

            time.sleep(wave_speed)

        while(servos.servo[2].anlge > elbow_flexation_1):
            servos.servo[2].angle -= 1

            time.sleep(wave_speed)

    print("Lowering arm!")

    #lower arm after wave and close gripper
    while(servos.servo[1].angle < SERVO1_NEUTRAL or servos.servo[2].angle > SERVO2_NEUTRAL):
        #add or subtract but make sure target isn't overshot
        servos.servo[1].angle = servos.servo[1].angle + 1 if servos.servo[1].angle < SERVO1_NEUTRAL else SERVO1_NEUTRAL
        servos.servo[2].angle = servos.servo[2].angle - 1 if servos.servo[2].angle > SERVO2_NEUTRAL else SERVO2_NEUTRAL
        servos.servo[3].angle = servos.servo[3].angle - 1 if servos.servo[3].angle > SERVO3_NEUTRAL else SERVO3_NEUTRAL

        time.sleep(arm_rase_lower_speed)

    print("Done waving! Goodbye!")


def move_to_neutral():
    #specify we will be editing global variable
    global servos

    #moves all servos to neutral position 1 at a time, with time in between to settle
    print("Setting servos to neutral position")

    servos.servo[0] = SERVO0_NEUTRAL
    time.sleep(2)
    print("Servo 1 at neutral position")

    servos.servo[1] = SERVO1_NEUTRAL
    time.sleep(2)
    print("Servo 2 at neutral position")

    servos.servo[2] = SERVO2_NEUTRAL
    time.sleep(2)
    print("Servo 3 at neutral position")

    servos.servo[3] = SERVO3_NEUTRAL
    time.sleep(2)
    print("Servo 4 at neutral position")





