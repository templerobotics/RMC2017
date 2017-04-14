#!/usr/bin/env python

import pygame
import cv2
from socket import *
from time import sleep
from matplotlib import pyplot as plt

img_status = 0




#message we want to send to target that will get parsed into command
imgstring = "img"
#target IP (intel nuc)
targetIP = '192.168.1.101'
port_send = 15000

sendingSocket = socket(AF_INET,SOCK_DGRAM)
targetAddress = (targetIP,port_send)

#recieving port
port_recv = 16000
recvSocket = socket(AF_INET, SOCK_DGRAM)
recvSocket.bind(('', port_recv))


#pygame stuff
pygame.joystick.init()
pygame.init()

j = pygame.joystick.Joystick(0)
j.init()


def grabJoystickData():
    pygame.event.get()

    img_status = j.get_button(5)

while True:
    grabJoystickData()
    print(img_status)
    if img_status == 1.0:
        print(img_status)
        sendingSocket.sendto(imgstring, targetAddress)
        sleep(0.5)
    #imgdata, address = recvSocket.recvfrom(1024)
    #plt.imshow(imgdata)
    #plt.show()
