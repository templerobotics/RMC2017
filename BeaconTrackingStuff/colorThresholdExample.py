import cv2
import numpy as np
from time import sleep

cam = cv2.VideoCapture(0)

sleep(1)

lower_H = np.array([0, 0, 0], dtype = np.uint8)
upper_H = np.array([180, 255, 255], dtype = np.uint8)
lower_BR = np.array([110, 150, 50], dtype = np.uint8)
upper_BR = np.array([130, 255, 255], dtype = np.uint8)
lower_G = np.array([50, 50, 50], dtype = np.uint8)
upper_G = np.array([70, 255, 255], dtype = np.uint8)

cv2.namedWindow('Test', cv2.WINDOW_NORMAL)

while True:

    _, frame = cam.read()

    hsvBG = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsvR = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    maskH = cv2.inRange(hsvBG, lower_H, upper_H)
    maskB = cv2.inRange(hsvBG, lower_BR, upper_BR)
    maskG = cv2.inRange(hsvBG, lower_G, upper_G)
    maskR = cv2.inRange(hsvR, lower_BR, upper_BR)

    bThresh = cv2.bitwise_and(frame, frame, mask = maskB)
    gThresh = cv2.bitwise_and(frame, frame, mask = maskG)
    rThresh = cv2.bitwise_and(frame, frame, mask = maskR)

    bgThresh = cv2.bitwise_or(bThresh, gThresh, mask = maskH)
    bgrThresh = cv2.bitwise_or(bgThresh, rThresh, mask = maskH)

    #bgrThresh = cv2.dilate(bgrThresh, None, iterations = 2)
    #bgrThresh = cv2.erode(bgrThresh, None, iterations = 4)

    cv2.imshow('Test', bgrThresh)

    if cv2.waitKey(30) == 27:
        break

cv2.destroyAllWindows()
cam.release()
