#!/usr/bin/env python

import pygame as pyg
import cv2
from socket import *
from time import sleep

import pygame
from time import sleep
from socket import *


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
    

    data["joyX"] = j.get_axis(0)
    data["joyY"] = j.get_axis(1)
    data["triggers"] = j.get_axis(2)
    #data["rightY"] = j.get_axis(3)
    #data["rightX"] = j.get_axis(4)
    #data["button1"] = j.get_button(0)
    #data["button2"] = j.get_button(1)
    #data["button3"] = j.get_button(2)
    #data["button4"] = j.get_button(3)
    #data["button5"] = j.get_button(4)
    #data["button6"] = j.get_button(5)
    #data["button7"] = j.get_button(6)
    #data["button8"] = j.get_button(7)
    #data["button9"] = j.get_button(8)
    #data["button10"] = j.get_button(9)
    #data["button11"] = j.get_button(10)
    #data["button12"] = j.get_button(11)

while True:
    try:
        grabJoystickData()
        message = toString(data)
        print(message)
        clientSocket.sendto(message, serverAddress)
        sleep(0.3)
    #Trying to get it to saftely exit...
    except (KeyboardInterrupt):
        clientSocket.shutdown()
        clientSocket.close()
        pygame.quit()
        pygame.QUIT()
        break





#Mat  img = Mat::zeros( height,width, CV_8UC3);
#   int  imgSize = img.total()*img.elemSize();
#   uchar sockData[imgSize];

# //Receive data here

#   for (int i = 0; i < imgSize; i += bytes) {
#   if ((bytes = recv(connectSock, sockData +i, imgSize  - i, 0)) == -1) {
#     quit("recv failed", 1);
#    }
#   }

# // Assign pixel value to img

# int ptr=0;
# for (int i = 0;  i < img.rows; i++) {
#  for (int j = 0; j < img.cols; j++) {                                     
#   img.at<cv::Vec3b>(i,j) = cv::Vec3b(sockData[ptr+ 0],sockData[ptr+1],sockData[ptr+2]);
#   ptr=ptr+3;
#   }
#  }