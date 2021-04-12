# TEA@UCF Senior Design Mechatronics Project
# Provides method for performer to remotely control animatronic. 
# Julia Warner 2/25/21

#debug level to control amount of print statements
#should typically be run on debug 1
DEBUG_LEVEL = 2

#python library used to create sockets
import socket
import keyboard

#constants for IP addresses and port number
RASPI_IP = '192.168.0.180'
PORT_NUM = 14357

torso_moving_right = False
torso_moving_left = False

#sets up keyboard listener objects used to send commands
def link_keys():
    #link y key to yes method
    keyboard.on_press_key("y", action_yes)

    #link h key to happy method
    keyboard.on_press_key("h", action_happy)

    #link s key to sad method
    keyboard.on_press_key("s", action_sad)

    #link a key to angry method
    keyboard.on_press_key("a", action_angry)

    #link c key to sad method
    keyboard.on_press_key("c", action_confused)

    #link x key to stop method
    keyboard.on_press_key("x", action_stop)

    #link right arrow key down to move torso right
    keyboard.on_press_key("right", action_torso_right)

    #link left arrow key down to move torso left
    keyboard.on_press_key("left", action_torso_left)

    #link right arrow key up to stop torso
    keyboard.on_release_key("right", action_torso_stop)

    #link left arrow key up to stop torso
    keyboard.on_release_key("left", action_torso_stop)

    #link g key to greeting
    keyboard.on_press_key("g", action_greeting)

    #link r key to reaching forward
    keyboard.on_press_key("r", action_reach_forward)

    #link t key to reaching backwards
    keyboard.on_press_key("t", action_reach_back)

    #link d key to giving diploma
    keyboard.on_press_key("d", action_give_diploma)

    #link n key to making neutral face
    keyboard.on_press_key("n", action_neutral)

    #link q key to homing the servos
    keyboard.on_press_key("q", action_home_servos)

######### ACTION FUNCTIONS #########

#sends "yes" command to Raspberry Pi
def action_yes(key_event):
    print("\nyes!")
    connection.send(bytes("y", "utf-8"))

#sends "happy" command to Raspberry Pi
def action_happy(key_event):
    print("\nhappy!")
    connection.send(bytes("h", "utf-8"))

#sends "sad" command to Raspberry Pi
def action_sad(key_event):
    print("\nsad!")
    connection.send(bytes("s", "utf-8"))

#sends "angry" command to Raspberry Pi
def action_angry(key_event):
    print("\nangry!")
    connection.send(bytes("a", "utf-8"))

#sends "confused" command to Raspberry Pi
def action_confused(key_event):
    print("\nconfused!")
    connection.send(bytes("c", "utf-8"))

#sends "stop" command to Raspberry Pi
def action_stop(key_event):
    #change global running variable to false
    global running
    print("\nsending stop!")
    connection.send(bytes("x", "utf-8"))
    running = False

#sends rotate torso right command to Raspberry Pi
def action_torso_right(key_event):
    global torso_moving_right
    global torso_moving_left
    
    #only send the signal if the torso isn't already moving
    if not torso_moving_right:
        torso_moving_right = True
        print("\nmoving torso right!")
        connection.send(bytes("tr", "utf-8"))

#sends rotate torso left command to Raspberry Pi
def action_torso_left(key_event):
    global torso_moving_right
    global torso_moving_left

    #only send the signal if the torso isn't already moving
    if not torso_moving_left:
        torso_moving_left = True
        print("\nmoving torso left!")
        connection.send(bytes("tl", "utf-8"))

#sends stop rotating torso command to Raspberry Pi
def action_torso_stop(key_event):
    global torso_moving_right
    global torso_moving_left

    torso_moving_right = False
    torso_moving_left = False

    print("\nstopping torso rotation!")
    connection.send(bytes("ts", "utf-8"))

#sends greeting command to Raspberry Pi
def action_greeting(key_event):
    print("\ngreeting!")
    connection.send(bytes("g", "utf-8"))

#sends reach forwards command to Raspberry Pi
def action_reach_forward(key_event):
    print("\nreaching out!")
    connection.send(bytes("rf", "utf-8"))

#sends reach backwards command to Raspberry Pi
def action_reach_back(key_event):
    print("\nreaching back!")
    connection.send(bytes("rb", "utf-8"))

#sends give diploma command to Raspberry Pi
def action_give_diploma(key_event):
    print("\ngiving diploma!")
    connection.send(bytes("d", "utf-8"))

#sends neutral command to Raspberry Pi
def action_neutral(key_event):
    print("\nneutral!")
    connection.send(bytes("n", "utf-8"))

#sends home servos to Raspberry Pi
def action_home_servos(key_event):
    print("\nhoming servos!!")
    connection.send(bytes("q", "utf-8"))


######### MESSAGE PROCESSING FUNCTIONS #########

#Processes a message received from the Raspberry Pi.
#Checks if any functions need to be called as a result of message contents, 
#otherwise prints informational message


######### MAIN SCRIPT CODE #########

#create socket object
connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

#notify attempting to connect
if DEBUG_LEVEL > 0:
    print("Attempting to connect to Raspberry Pi...")

#connect to raspberry pi
connection.connect((RASPI_IP, PORT_NUM))

#print success message
if DEBUG_LEVEL > 0:
    print(f"Connection to Raspberry Pi on {RASPI_IP} established!")

#set timeout length to low number to create non-blocking socket
connection.settimeout(0.25)

#setup control keys
link_keys()

#loop
running = True
while running :
    #check if message new message from Raspberry Pi
    try:
        raw_msg = connection.recv(1024)
    except socket.timeout:
        #socket timed out, no new message
        pass
    else:
        #message found, process message
        msg = raw_msg.decode("utf-8")



#end connection when loop ends
connection.close()

print("Connection closed, thank you!")

