#!/usr/bin/env python

import cv2 #OpenCV
import time
from socket import * #Networking Stuff
import winsound #to play sound effects
import threading

#import pyfly2
#import MotorControl as mc
s = 0
audioplay = True

#OpenCV int webcam
port = int(raw_input("Camera Port: "))
cam = cv2.VideoCapture(port)
time.sleep(0.5)
cam.release()
cam.open(port)
time.sleep(0.5)

#Pi/Motor Controller's IP and Port
targetIP = '192.168.1.101'
port = 12000


#Init socket
clientSocket = socket(AF_INET,SOCK_DGRAM)
serverAddress = (targetIP, port)

detect = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

def play():
    global s
    while True:
        if (s == 1):
            winsound.PlaySound('fire.wav',winsound.SND_FILENAME)
            print s
        if (audioplay == False):
            break

def distanceFormula(x1, y1, x2, y2):
    return Math.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)

def getDistanceFromCenter(box):
    return distanceFormula(320, 240, box[0] + box[2], box[1] + box[3])

def getClosest(boxes):
    if boxes == None:
        return (-1, -1, -1, -1)
    elif len(boxes) == 1:
        return boxes[0]
    else:
        closest = boxes[0]
        distance = getDistanceFromCenter(boxes[0])
        for box in boxes:
            d = getDistanceFromCenter(box)
            if d < distance:
                distance = d
                closest = box
        return closest

#Load frame, apply front face haar classifier, and check to see if identified rectangle is in center of frame/center of gun
def processframe():
    global s
    #grab frame
    retval, frame = cam.read(
        
    #Apply the frontal classifier:
    frontal = detect.detectMultiScale(frame,scaleFactor=1.3, minNeighbors=5, minSize=(30, 30), flags=cv2.CASCADE_SCALE_IMAGE)

    (x, y, w, h) = getClosest(frontal)
    
    #where (x,y) is upper vertex of rect. w/h is width/height
    #center of rectangle, draw a circle
    center = ( (x+(w/2)), (y+(h/2)))
    cv2.rectangle(frame, (x, y), (x+w, y+h), (0,0,153), 5)	
    cv2.circle(frame, (center), 3, (0, 200, 0), -1)
    #Grab center of rectangle (only x is needed) and test to see if it falls within
    center = ( (x+(w/2)))
            #print(center)
    #Send pysocket commands to motorcontroller/pi regarding situation
    if (center < 390 and center > 250 ):
        cv2.putText(frame, "Target Found", (20, 30), cv2.FONT_HERSHEY_DUPLEX, 0.5,(27, 204, 4), 2)
        clientSocket.sendto('shoot'.encode(),serverAddress)
        s = 1
    elif (center > 390):
        clientSocket.sendto('right'.encode(),serverAddress)
        s = 0
    elif (center < 250):
        clientSocket.sendto('left'.encode(),serverAddress)
        s = 0
    cv2.imshow("hurrderr", frame)

time.sleep(1)
#Initial command to get gun moving
clientSocket.sendto('right'.encode(),serverAddress)

sound_thread = threading.Thread(target=lambda:play())
sound_thread.start()
while True:
    processframe()
    if cv2.waitKey(30) == 27:
        clientSocket.sendto('stopTurning'.encode(),serverAddress)
        clientSocket.sendto('stopShooting'.encode(),serverAddress)
        audioplay = False
        break
cam.release()
cv2.DestroyAllWindows()
