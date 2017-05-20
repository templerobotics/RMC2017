#!/usr/bin/env python

import cv2
from time import sleep
from collections import deque
from turmc.motor_control.camera_stepper import step2deg

def BeaconTracking2017(color, camera, stepper):

    #This line and the following two while loops ensure the stepper is at position 0
    stepper.goto(0)

    while stepper.isMoving():
        sleep(0.2)

    while stepper.getPosition() > 0:
        stepper.stepRight()
        sleep(0.1)

    data = deque()

    #Captures 400 images in 40 seconds; storing position data alongside them
    data.append({'image': camera.getImage(), 'position': 0})
    while stepper.getPosition() < 399:
        stepper.stepLeft()
        data.append({'image': camera.getImage(), 'position': stepper.getPosition()})
        sleep(0.1)

    angles = []
    for item in data:
        #Find colored contour
        contour = findColoredContour(item['image'], color)

        if not contour == None:
            center = findCenter(contour)
            yaw = getAnglesToPoint(center, camera.matrix)[0]
            angles.append(step2deg(item['position']) - yaw)

    meanBearing = sum(angles) / len(angles)

    return meanBearing
