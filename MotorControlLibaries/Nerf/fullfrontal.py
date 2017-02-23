#!/usr/bin/env python

import cv2
import time
from socket import *

#import pyfly2
#import MotorControl as mc

#Motor Functions


def getArea(rectangle):
    return rectangle[2] * rectangle[3]




def processframe():
    retval, frame = cam.read()
    #frame = cv2.imread('img.png')
    #frame = camera.GrabNumPyImage(format='bgr')
    
    #frame = cv2.resize(frame,None,fx=0.5, fy=0.5, interpolation = cv2.INTER_AREA)
    detect = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
    upper = cv2.CascadeClassifier('haarcascade_frontalface_alt2.xml')
	#comments
    frontal = detect.detectMultiScale(frame,scaleFactor=1.3, minNeighbors=5, minSize=(30, 30), flags=cv2.CASCADE_SCALE_IMAGE)
    #Eventually will download and implement profile classifier
    profile = upper.detectMultiScale(frame,scaleFactor=1.3, minNeighbors=5, minSize=(30, 30), flags=cv2.CASCADE_SCALE_IMAGE)

    
    for (x,y,w,h) in frontal:
        
        
        center = ( (x+(w/2)), (y+(h/2)))
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0,0,153), 5)	
        cv2.circle(frame, (center), 3, (0, 200, 0), -1)
        center = ( (x+(w/2)))
		#print(center)
        if (center < 390 and center > 250 ):
            cv2.putText(frame, "Target Found", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0, 0, 255), 2)
            clientSocket.sendto('shoot'.encode(),serverAddress)
        elif (center > 390):
            clientSocket.sendto('right'.encode(),serverAddress)
        elif (center < 250):
            clientSocket.sendto('left'.encode(),serverAddress)
    cv2.imshow("w", frame)


        
#context = pyfly2.Context()
#if context.num_cameras:
#    camera = context.get_camera(0)
#    camera.Connect()
#    camera.StartCapture()

targetIP = '192.168.1.102'
port = 12000

clientSocket = socket(AF_INET,SOCK_DGRAM)
serverAddress = (targetIP, port)



STATE_TARGET_FOUND = False
STATE_PANNING = False
DEBUG_MODE = True
currentPanAngle = 90





port = int(raw_input("Camera Port: "))
cam = cv2.VideoCapture(port)

cam.release()
cam.open(port)
time.sleep(5)
clientSocket.sendto('right'.encode(),serverAddress)


while True:
	processframe()
	if cv2.waitKey(30) == 27 & 0xFF == ord('z'):
		break
cam.release()
cv2.DestoryAllWindows()

