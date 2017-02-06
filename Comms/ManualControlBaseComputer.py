import pygame
from time import sleep
from socket import *
from formattingTools import *

targetIP = '192.168.1.132'
port = 5555

pygame.joystick.init()
pygame.init()

j = pygame.joystick.Joystick(0)
j.init()

clientSocket = socket(AF_INET, SOCK_DGRAM)
serverAddress = (targetIP, port)

data = {}

def grabJoystickData():
    pygame.event.get()

    data["joyX"] = j.get_axis(0)
    data["joyY"] = j.get_axis(1)
    data["button1"] = j.get_button(0)
    data["button2"] = j.get_button(1)
    data["button3"] = j.get_button(2)
    data["button4"] = j.get_button(3)
    data["button5"] = j.get_button(4)
    data["button6"] = j.get_button(5)
    data["button7"] = j.get_button(6)
    data["button8"] = j.get_button(7)
    data["button9"] = j.get_button(8)
    data["button10"] = j.get_button(9)

while True:
    grabJoystickData()
    message = toString(data)
    #print(message)
    clientSocket.sendto(message, serverAddress)
    sleep(0.2)
