# TEA@UCF Senior Design Mechatronics Project
# Julia Warner
# Tests connection between Raspberry Pi and controller computer using sockets. 

#python library used to create sockets
import socket

#constants for IP addresses and port number
RASPI_IP = '192.168.2.9'
PORT_NUM = 14357

#create socket object for self
rasPiSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

#bind socket to own IP address and random port
rasPiSocket.bind((RASPI_IP, 14357))

#set buffer
rasPiSocket.listen(5)

#accept connection as new socket
connection, address = rasPiSocket.accept()

#print IP of controller computer
print(f"A connection to controller with IP {address} has been establised!")

#start listening for controller input
running = True
while running:
    #receive message from the controller
    rawMsg = connection.recv(1024)
    msgString = rawMsg.decode("utf-8")

    #print contoller message
    print("Recieved controller input " + msgString)

    #send back response confirming input
    connection.send(bytes('I heard ' + msgString, "utf-8"))

    #check if test should end
    if msgString == 'stop':
        running = False
        connection.send(bytes('. Closing connection', "utf-8"))

#end connection when loop ends
connection.close()

