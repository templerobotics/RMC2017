#!/usr/bin/env python

import cv2
import time
import pigpio
import MotorControl as MC
#import pyfly2
#import MotorControl as mc

#Motor Functions
def startShooting():
    PI.write(FIRING_PIN, pigpio.HIGH)
    print("Firing")

def stopShooting():
    PI.write(FIRING_PIN, pigpio.LOW)
    print("Not firing")

def turnRight():
    panMotor.setSpeed(-1.0)
    print("Turning right")

def turnLeft():
    panMotor.setSpeed(1.0)
    print("Turning left")

def stopTurning():
    panMotor.setSpeed(0.0)
    print("Stopping turning")

def getArea(rectangle):
    return rectangle[2] * rectangle[3]



def initCam():
    port = int(raw_input("Camera Port: "))
    cam = cv2.VideoCapture('WIN_20170116_16_17_18_Pro.mp4')
    #cam.release()
    #cam.open(port)
    time.sleep(5)



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
            stopTurning()
            startShooting()
        elif (center > 390):
            stopShooting()
            turnRight()
        elif (center < 250):
            stopShooting()
            turnLeft()
        else:
            stopShooting()
            stopTurning()
    cv2.imshow("w", frame)


        
#context = pyfly2.Context()
#if context.num_cameras:
#    camera = context.get_camera(0)
#    camera.Connect()
#    camera.StartCapture()

PI = pigpio.pi()

FIRING_PIN = 6
PAN_PIN = 12
#TILT_PIN = 18

STATE_TARGET_FOUND = False
STATE_PANNING = False
DEBUG_MODE = True
currentPanAngle = 90

panMotor = MC.VexMotorModule(PAN_PIN)



port = int(raw_input("Camera Port: "))
cam = cv2.VideoCapture(port)

cam.release()
cam.open(port)
time.sleep(5)


while True:
	processframe()
	if cv2.waitKey(30) == 27 & 0xFF == ord('z'):
		break
cam.release()
cv2.DestoryAllWindows()


