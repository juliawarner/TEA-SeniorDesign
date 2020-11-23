# TEA@UCF Senior Design Mechatronics Project
# Julia Warner
# Tests connection between Raspberry Pi and controller computer using sockets. 

#python library used to create sockets
import socket

#constants for IP addresses and port number
RASPI_IP = '192.168.2.9'
PORT_NUM = 14357

#create socket object
connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

#connect to raspberry pi
connection.connect((RASPI_IP, 14357))

#print success message
print(f"Connection to Raspberry Pi on {RASPI_IP} established!")

#start waiting for user input
running = True
while(running):
    #get user input
    userInput = input()

    #send message to the raspberry pi
    connection.send(bytes(userInput, "utf-8"))

    #receive message from raspberry pi
    rawResponse = connection.recv(1024)
    response = rawResponse.decode("utf-8")

    #print response
    print(f'The Raspberry Pi said: {response}')

    #check if raspberry pi closed the connection
    if(response == 'Closing connection'):
        print('Closing controller test program')
        running = False

#end connection when loop ends
connection.close()