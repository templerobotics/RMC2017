#!/usr/bin/env python

import pygame
import cv2
from socket import *
from time import sleep
from matplotlib import pyplot as plt

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

    imgbool = j.get_button(0)


while True:
    try:
        grabJoystickData()
        if imgbool == True:
            print(imgbool)
            sendingSocket.sendto(imgstring, targetAddress)
            sleep(0.3)
        imgdata, address = serverSocket.recvfrom(1024)
        plt.imshow(imgdata)
        plt.show()
        

    ##Trying to get it to saftely exit...
    #except (KeyboardInterrupt):
    #    clientSocket.shutdown()
    #    clientSocket.close()
    #    pygame.quit()
    #    pygame.QUIT()
    #    break





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