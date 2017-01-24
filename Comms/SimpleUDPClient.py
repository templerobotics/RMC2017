from time import *
from socket import *
from formattingTools import *

clientSocket = socket(AF_INET, SOCK_DGRAM)
serverAddress = ("192.168.2.132", 12000)

data = {"Things can only get better from here":1}

message = toString(data)

while True:
    clientSocket.sendto(message, serverAddress)
    sleep(1)
