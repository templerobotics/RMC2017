#!/usr/bin/env python

import cv2
from time import sleep
from collections import deque

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
        stepper.stepRight()
        data.append({'image': camera.getImage(), 'position': stepper.getPosition()})
        sleep(0.1)

    angles = []
    for item in data:
        #Process the image and calculate the adjusted angle to target for BOTH colors
        angles.append(5)

    meanBearing = sum(angles) / len(angles)

    return meanBearing
