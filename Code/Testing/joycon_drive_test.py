from evdev import InputDevice, categorize, ecodes
import RPi.GPIO as GPIO
import time

#motor GPIO setup
m1_current_speed = 25 #default speed is 25% of duty cycle (low)
m2_current_speed = 25
fast_speed = 100 #100% of duty cycle, zoom
slow_speed = 50 #50% of duty cycle, slow for turning

#create object to store joycon data
joycon = InputDevice('/dev/input/event11')

#joycon detect joystick movement cutoffs
JOYCON_LEFT_CUTOFF = 900
JOYCON_RIGHT_CUTOFF = -900
JOYCON_FORWARD_CUTOFF = -900
JOYCON_BACKWARD_CUTOFF = 900

#joystick movement tracking
#number of second where movement IS NOT detected before shutting off
drive_event_counter_max = 0.5
#tracks consecutive loops where [forward, backward, left, right] is NOT detected
drive_event_counter = [drive_event_counter_max, drive_event_counter_max, drive_event_counter_max, drive_event_counter_max]
COUNTER_F_INDEX = 0
COUNTER_B_INDEX = 1
COUNTER_L_INDEX = 2
COUNTER_R_INDEX = 3
#array to keep track of current movement
drive_current_movement = [False, False, False, False] #[forward, backward, left, right]
#unsigned integers to compare with evdev objects variables
UPDOWN_CODE = 2
RIGHTLEFT_CODE = 3 



def forward():
    print("Moving forwards!")

def reverse():
    print("Reversing!")
    
def right():
    print("Turning right!")
    
def left():
    print("Turning left!")
    
def forward_right():
    print("Forward right!")
    
def forward_left():
    print("Forward left!")
    
def reverse_right():
    print("Reverse right!")
    
def reverse_left():
    print("Reverse left!")

def stop():
    print("Stopping")

#looks at the global movement array and switches to correct drive function
def update_drive_movement():
    global drive_event_counter
    global drive
    
    #print(f"Drive movement counter: {drive_event_counter}")
    
    #check if we need to update current movement
    #this will happen when a counter hits its max value
    update = False
    for i in range(0, 4):
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
        if drive_current_movement == [False, False, False, False]:
            stop()
        elif drive_current_movement == [True, False, False, False]:
            forward()
        elif drive_current_movement == [True, False, True, False]:
            forward_left()
        elif drive_current_movement == [True, False, False, True]:
            forward_right()
        elif drive_current_movement == [False, True, False, False]:
            reverse()
        elif drive_current_movement == [False, True, True, False]:
            reverse_left()
        elif drive_current_movement == [False, True, False, True]:
            reverse_right()
        elif drive_current_movement == [False, False, True, False]:
            left()
        elif drive_current_movement == [False, False, False, True]:
            right()

#print out info at start
print(joycon)
t0 = time.time()

while(1):
    print("Loop")
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
            elif(event.code == RIGHTLEFT_CODE):
                #filter by left or right and value
                if(event.value < JOYCON_RIGHT_CUTOFF):
                    drive_event_counter[COUNTER_R_INDEX] = 0
                    
                    #swiftly disable left movement
                    drive_event_counter[COUNTER_L_INDEX] = drive_event_counter_max + 1
                elif(event.value > JOYCON_LEFT_CUTOFF):
                    drive_event_counter[COUNTER_L_INDEX] = 0
                    
                    #swiftly disable right movement
                    drive_event_counter[COUNTER_R_INDEX] = drive_event_counter_max + 1
                
        #print("Loop")
                    
        #update time
        dt = time.time()-t0
        t0 = time.time()
        for i in range(0, 4):
            drive_event_counter[i] += dt
                    
        update_drive_movement()
            
    print("Doing other stuff now")


    