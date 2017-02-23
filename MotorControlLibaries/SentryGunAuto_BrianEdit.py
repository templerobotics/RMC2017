import pigpio
import MotorControl as MC

import cv2

from time import sleep

PI = pigpio.pi()

FIRING_PIN = 6
PAN_PIN = 12
#TILT_PIN = 18

STATE_TARGET_FOUND = False
STATE_PANNING = False
DEBUG_MODE = True


frontalhaar = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
cam = cv2.VideoCapture(0)

cv2.namedWindow('Window', cv2.WINDOW_NORMAL)

currentPanAngle = 90
#currentTiltAngle = 90

panMotor = MC.VexMotorModule(PAN_PIN)

def getArea(rectangle):
    return rectangle[2] * rectangle[3]

def getCenterOfTarget(rectangle):
    x = int(rectangle[0] + 0.5 * rectangle[2])
    y = int(rectangle[1] + 0.5 * rectangle[3])
    return (x, y)

#Sometimes the camera shuts itself off, this is to restart it
def resetCamera():
    cam.release()
    cam.open(0)
    return True

#Searches image for haar classifier and returns the center of the largest target
def checkFrameForTarget(image):

    #Frontal Face detection
    frontal_rect = frontalhaar.detectMultiScale(frame,scaleFactor=1.3, minNeighbors=4, minSize=(30, 30), flags=cv2.CASCADE_SCALE_IMAGE)

    if len(frontal_rect) == 0:
        return False, (0, 0)

    F_largestBox = frontal_rect[0]
    F_largestArea = getArea(F_largestBox)

    for F_box in frontal_rect:
        F_area = getArea(F_box)
        if F_area > F_largestArea:
            F_largestBox = F_box
            F_largestArea = F_area

    return True, F_largestBox
    
    





def startShooting():
    PI.write(FIRING_PIN, pigpio.HIGH)
    print("Firing")

def stopShooting():
    PI.write(FIRING_PIN, pigpio.LOW)
    print("Not firing")

def turnRight():
    panMotor.setSpeed(1.0)
    print("Turning right")

def turnLeft():
    panMotor.setSpeed(-1.0)
    print("Turning left")

def stopTurning():
    panMotor.setSpeed(0.0)
    print("Stopping turning")

def main():

    #Main loop
    while True:

        #Grab image from camera
        retval, frame = cam.read()

        #Restarts the main loop if the camera wasn't working
        if len(frame) == 0:
            resetCamera()
            break

        #Processes image for a target
        targetFound, targetCoords = checkFrameForTarget(frame)

        if targetFound:
            #Target is in the right tridant
            if targetCoords[0] > rightDeadzoneMargin:
                stopShooting()
                turnRight()
            #Target is in the left tridant
            elif targetCoords[0] < leftDeadzoneMargin:
                stopShooting()
                turnLeft()
            #Target is in the middle tridant
            else:
                stopTurning()
                startShooting()
        else:
            stopShooting()
            stopTurning()

        #Draws a window with a dot on the center of the target
        if DEBUG_MODE:
            cv2.circle(frame, targetCoords, 3, (0, 0, 255), -1)
            cv2.imshow('Window', frame)
            if cv2.waitKey(30) == 27:
                break

if __name__ == '__main__':
    while True:
        main()
