#!/usr/bin/env python

#Sam Wilson 3/8/2017

import cv2
import numpy as np
import math
from time import sleep
from CameraConstants import *

#Make sure to check if the cameras have the correct number
#You may have to swap these values if they are backwards
ted_port = 1
bill_port = 2

#This script requires this image file or bad things will happen
error_image = cv2.imread('errors.png')

#Distance between the cameras in cm
k = 12
#Distance between the targets in cm
K = 12.7

def medianFormula( leg_m, leg_n, base):
    #1/2 * sqrt(2*m^2 + 2*n^2 - k^2)
    try:
        return 0.5 * math.sqrt(2 * leg_m ** 2 + 2 * leg_n ** 2 - base ** 2)
    except:
        print("Complex value of median")
        print("leg_m = " + str(leg_m))
        print("leg_n = " + str(leg_n))
        print("base = " + str(base))
        return 0

#Angles should be in DEGREES
def getRobotCoordinates( k, K, alpha, beta, gamma, delta):

    if alpha == 0 or beta == 0 or gamma == 0 or delta == 0:
        return (0, 0)

    #Corrects exterior angles
    beta = 180 - beta
    delta = 180 - delta

    #Solve for the 3rd angle of each triangle
    ab = math.radians(180 - alpha - beta)
    gd = math.radians(180 - gamma - delta)
    
    #Convert angles to radians
    alpha = math.radians(alpha)
    beta = math.radians(beta)
    gamma = math.radians(gamma)
    delta = math.radians(delta)

    #Calculate legs of minor triangles
    a = (k * math.sin(beta)) / math.sin(ab)
    b = (k * math.sin(alpha)) / math.sin(ab)
    c = (k * math.sin(delta)) / math.sin(gd)
    d = (k * math.sin(gamma)) / math.sin(gd)

    #Calculate medians of minor triangles
    M = medianFormula( a, b, k)
    N = medianFormula( c, d, k)

    #print('--')
    #print(M)
    #print(N)
    #Calculate median of major triangle
    P = medianFormula( M, N, K)

    #Calculates semiperimeter and area of major triangle
    g = (M + N + K) / 2.0
    try:
        A = math.sqrt( g * (g - M) * (g - N) * (g - K) )
    except ValueError:
        #print("Heron Complex")
        #print("M: " + str(M))
        #print("N: " + str(N))
        A = 0.0

    #X coordinate
    J = 2.0 * A / K

    #Y coordinate
    S = math.sqrt(P ** 2 - J ** 2)

    #Sign correction of Y coordinate
    if M > N:
        S = S * -1

    return (J, S)

#Returns the contours from a colored image
def getContours(image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    contours, _ = cv2.findContours(image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours

#Returns the largest contour from a given set
def getLargestContour(image):
    contours = getContours(image)
    largestA = -1.0
    try:
        largestC = contours[0]
    except:
        return None
    for c in contours:
        area = cv2.contourArea(c)
        if area > largestA:
            largestA = area
            largestC = c
    return largestC

#Calculates the center of a contour
def findCenter(contour):
    M = cv2.moments(contour)
    x = int(M['m10'] / M['m00'])
    y = int(M['m01'] / M['m00'])
    return (x, y)

#Calculates angle by using the focal length and pixel position
def getAnglesToPoint(point, camMatrix):
    yaw = 90 - math.degrees(math.atan((point[0] - 319) / camMatrix[0, 0]))
    #pitch = 90 - math.degrees(math.atan(point[1] - camMatrix[1, 2]) / camMatrix[1, 1]))
    return (yaw, 0)
    

scam = cv2.VideoCapture(ted_port)
pcam = cv2.VideoCapture(bill_port)

sleep(2)

lower_R = np.array([120, 120, 30], dtype = np.uint8)
upper_R = np.array([135, 255, 255], dtype = np.uint8)
lower_G = np.array([40, 50, 30], dtype = np.uint8)
upper_G = np.array([60, 255, 255], dtype = np.uint8)

#cv2.namedWindow('Red', cv2.WINDOW_NORMAL)
#cv2.namedWindow('Green', cv2.WINDOW_NORMAL)
cv2.namedWindow('Port', cv2.WINDOW_NORMAL)
cv2.namedWindow('Star', cv2.WINDOW_NORMAL)

while True:
    _, Sframe = scam.read()
    _, Pframe = pcam.read()

    if Sframe == None:
        Sframe = error_image

    if Pframe == None:
        Pframe = error_image

    Sframe = cv2.undistort(Sframe, tedMatrix, tedDistort)
    Pframe = cv2.undistort(Pframe, billMatrix, billDistort)

    ShsvG = cv2.cvtColor(Sframe, cv2.COLOR_BGR2HSV)
    ShsvR = cv2.cvtColor(Sframe, cv2.COLOR_RGB2HSV)
    PhsvG = cv2.cvtColor(Pframe, cv2.COLOR_BGR2HSV)
    PhsvR = cv2.cvtColor(Pframe, cv2.COLOR_RGB2HSV)

    SmaskG = cv2.inRange(ShsvG, lower_G, upper_G)
    SmaskR = cv2.inRange(ShsvR, lower_R, upper_R)
    PmaskG = cv2.inRange(PhsvG, lower_G, upper_G)
    PmaskR = cv2.inRange(PhsvR, lower_R, upper_R)

    SgThresh = cv2.bitwise_and(Sframe, Sframe, mask = SmaskG)
    SrThresh = cv2.bitwise_and(Sframe, Sframe, mask = SmaskR)
    PgThresh = cv2.bitwise_and(Pframe, Pframe, mask = PmaskG)
    PrThresh = cv2.bitwise_and(Pframe, Pframe, mask = PmaskR)

    PrYaw = 0
    PgYaw = 0
    SrYaw = 0
    SgYaw = 0

    try:
        SgContour = getLargestContour(SgThresh)
        SrContour = getLargestContour(SrThresh)

        SgCenter = findCenter(SgContour)
        SrCenter = findCenter(SrContour)

        SgYaw = getAnglesToPoint(SgCenter, tedMatrix)[0]
        SrYaw = getAnglesToPoint(SrCenter, tedMatrix)[0]

        cv2.drawContours(Sframe, [SgContour], -1, (0, 255, 0), 2)
        cv2.drawContours(Sframe, [SrContour], -1, (0, 0, 255), 2)

        cv2.circle(Sframe, SgCenter, 3, (0, 255, 0), -1)
        cv2.circle(Sframe, SrCenter, 3, (0, 0, 255), -1)

        cv2.putText(Sframe, str(SgYaw) + 'deg', SgCenter, cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0))
        cv2.putText(Sframe, str(SrYaw) + 'deg', SrCenter, cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255))
    except:
        print("")

    try:
        PgContour = getLargestContour(PgThresh)
        PrContour = getLargestContour(PrThresh)

        PgCenter = findCenter(PgContour)
        PrCenter = findCenter(PrContour)

        PgYaw = getAnglesToPoint(PgCenter, billMatrix)[0]
        PrYaw = getAnglesToPoint(PrCenter, billMatrix)[0]

        cv2.drawContours(Pframe, [PgContour], -1, (0, 255, 0), 2)
        cv2.drawContours(Pframe, [PrContour], -1, (0, 0, 255), 2)

        cv2.circle(Pframe, PgCenter, 3, (0, 255, 0), -1)
        cv2.circle(Pframe, PrCenter, 3, (0, 0, 255), -1)

        cv2.putText(Pframe, str(PgYaw) + 'deg', PgCenter, cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0))
        cv2.putText(Pframe, str(PrYaw) + 'deg', PrCenter, cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255))
    except:
        print("")

    coords = getRobotCoordinates(k, K, PrYaw, SrYaw, PgYaw, SgYaw)
    print(coords)
        
    cv2.imshow('Star', Sframe)
    cv2.imshow('Port', Pframe)

    if cv2.waitKey(30) == 27:
        break

cv2.destroyAllWindows()
scam.release()
pcam.release()
