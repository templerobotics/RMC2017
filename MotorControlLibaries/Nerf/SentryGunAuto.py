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

targetClassifier = cv2.CascadeClassifier("FullBodyHaarClassifier.xml")
camera = cv2.VideoCapture(0)

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
    camera.release()
    camera.open(0)
    return True

#Searches image for haar classifier and returns the center of the largest target
def checkFrameForTarget(image):

    rectangles = cv2.detectMultiScale(image, scaleFactor = 1.3, minNeighbors = 4, minSize = (30, 30))

    if len(rectangles) == 0:
        return False, (0, 0)

    largestBox = rectangles[0]
    largestArea = getArea(largestBox)

    for box in rectangles:
        area = getArea(box)
        if area > largestArea:
            largestBox = box
            largestArea = area

    return True, largestBox

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
        _, frame = camera.read()

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
