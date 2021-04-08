from evdev import InputDevice, categorize, ecodes
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM) #use BCM numbering for pins

fast_speed = 100 #100% of duty cycle, zoom
slow_speed = 50 #50% of duty cycle, slow starting to walk
accel_delay = 0.01 #10 milliseconds between each increase in speed

#motor GPIO setup
m1_in1 = 23
m1_in2 = 24
m1_en = 25
m1_current_speed = fast_speed
GPIO.setup(m1_in1,GPIO.OUT)
GPIO.setup(m1_in2,GPIO.OUT)
GPIO.setup(m1_en,GPIO.OUT)
GPIO.output(m1_in1,GPIO.LOW)
GPIO.output(m1_in2,GPIO.LOW)
m1_p=GPIO.PWM(m1_en,1000)
m1_p.start(m1_current_speed)

m2_in1 = 17
m2_in2 = 27
m2_en = 22
m2_current_speed = fast_speed
GPIO.setup(m2_in1,GPIO.OUT)
GPIO.setup(m2_in2,GPIO.OUT)
GPIO.setup(m2_en,GPIO.OUT)
GPIO.output(m2_in1,GPIO.LOW)
GPIO.output(m2_in2,GPIO.LOW)
m2_p=GPIO.PWM(m2_en,1000)
m2_p.start(m2_current_speed)

#create object to store joycon data
joycon = InputDevice('/dev/input/event12')

#joycon detect joystick movement cutoffs
#JOYCON_LEFT_CUTOFF = 900
#JOYCON_RIGHT_CUTOFF = -900
JOYCON_FORWARD_CUTOFF = -900
JOYCON_BACKWARD_CUTOFF = 900

#joystick movement tracking
#number of second where movement IS NOT detected before shutting off
drive_event_counter_max = 0.5
#tracks consecutive loops where [forward, backward, left, right] is NOT detected
#drive_event_counter = [drive_event_counter_max, drive_event_counter_max, drive_event_counter_max, drive_event_counter_max]
drive_event_counter = [drive_event_counter_max, drive_event_counter_max]
COUNTER_F_INDEX = 0
COUNTER_B_INDEX = 1
#COUNTER_L_INDEX = 2
#COUNTER_R_INDEX = 3
#array to keep track of current movement
#drive_current_movement = [False, False, False, False] #[forward, backward, left, right]
drive_current_movement = [False, False] #[forward, backward]
#unsigned integers to compare with evdev objects variables
UPDOWN_CODE = 2
#RIGHTLEFT_CODE = 3 



def forward():
    global m1_current_speed
    global m2_current_speed
    print("Moving forwards!")
    
    #move left motor forwards
    GPIO.output(m1_in1,GPIO.HIGH)
    GPIO.output(m1_in2,GPIO.LOW)
    
    #move right motor forwards
    GPIO.output(m2_in1,GPIO.HIGH)
    GPIO.output(m2_in2,GPIO.LOW)
    
    #ramp speed
#     print("Accelerating!")
#     while m1_current_speed < fast_speed:
#         m1_current_speed += 1
#         m2_current_speed += 1
#         
#         m1_p.ChangeDutyCycle(m1_current_speed)
#         m2_p.ChangeDutyCycle(m2_current_speed)
#         
#         time.sleep(accel_delay)
#     
#     print("At maximum speed")

def reverse():
    print("Reversing!")
    global m1_current_speed
    global m2_current_speed
    
    #move left motor backwardss
    GPIO.output(m1_in1,GPIO.LOW)
    GPIO.output(m1_in2,GPIO.HIGH)
    
    #move right motor backwards
    GPIO.output(m2_in1,GPIO.LOW)
    GPIO.output(m2_in2,GPIO.HIGH)
    
    #ramp speed
#     print("Accelerating!")
#     while m1_current_speed < fast_speed:
#         m1_current_speed += 1
#         m2_current_speed += 1
#         
#         m1_p.ChangeDutyCycle(m1_current_speed)
#         m2_p.ChangeDutyCycle(m2_current_speed)
#         
#         time.sleep(accel_delay)
    
#    print("At maximum speed")
    
# def right():
#     print("Turning right!")
#     
# def left():
#     print("Turning left!")
#     
# def forward_right():
#     print("Forward right!")
#     
# def forward_left():
#     print("Forward left!")
#     
# def reverse_right():
#     print("Reverse right!")
#     
# def reverse_left():
#     print("Reverse left!")

def stop():
    global m1_current_speed
    global m2_current_speed
    
    print("Stopping")
    
#    print("Slowing down!")
    
#     while m1_current_speed > slow_speed:
#         m1_current_speed -= 1
#         m2_current_speed -= 1
#         
#         m1_p.ChangeDutyCycle(m1_current_speed)
#         m2_p.ChangeDutyCycle(m2_current_speed)
#         
#         time.sleep(accel_delay)
        
#    print("Done slowing down!")
    
    #stop left motor
    GPIO.output(m1_in1,GPIO.LOW)
    GPIO.output(m1_in2,GPIO.LOW)
    
    #stop right motor
    GPIO.output(m2_in1,GPIO.LOW)
    GPIO.output(m2_in2,GPIO.LOW)
    

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
            stop()
        elif drive_current_movement == [True, False]:
            forward()
        elif drive_current_movement == [False, True]:
            reverse()

#print out info at start
print(joycon)
t0 = time.time()

while(1):
    #use evdec function to continuously read joycon inputs
    for event in joycon.read_loop():
        #filter by event type, print only button presses and joystick events
        if event.type == ecodes.EV_KEY:
            print(event)
            print(categorize(event))
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
#             elif(event.code == RIGHTLEFT_CODE):
#                 #filter by left or right and value
#                 if(event.value < JOYCON_RIGHT_CUTOFF):
#                     drive_event_counter[COUNTER_R_INDEX] = 0
#                     
#                     #swiftly disable left movement
#                     drive_event_counter[COUNTER_L_INDEX] = drive_event_counter_max + 1
#                 elif(event.value > JOYCON_LEFT_CUTOFF):
#                     drive_event_counter[COUNTER_L_INDEX] = 0
#                     
#                     #swiftly disable right movement
#                     drive_event_counter[COUNTER_R_INDEX] = drive_event_counter_max + 1
                
        #print("Loop")
                    
        #update time
        dt = time.time()-t0
        t0 = time.time()
        for i in range(0, 2):
            drive_event_counter[i] += dt
                    
        update_drive_movement()
            
    print("Doing other stuff now")


    