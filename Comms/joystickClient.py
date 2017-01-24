import pygame
from time import sleep
from socket import *
from formattingTools import *

pygame.joystick.init()
pygame.init()

j = pygame.joystick.Joystick(0)
j.init()

clientSocket = socket(AF_INET, SOCK_DGRAM)
serverAddress = ("192.168.2.132", 5555)

data = {}

while True:
    pygame.event.get()
    data["joyX"] = j.get_axis(0)
    data["joyY"] = j.get_axis(1)
    message = toString(data)
    print(message)
    clientSocket.sendto(message, serverAddress)
    sleep(0.2)
