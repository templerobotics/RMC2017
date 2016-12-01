#!/usr/bin/env python

import cv2

#Simplified grayscale function, assumes input is BGR
def toGray(image):
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

#Simplified gaussian blur, not to be confused with cv2.blur()
def blur(image, ksize = 5, sigmax = 0, sigmay = 0):
    return cv2.GaussianBlur(image, (ksize, ksize), sigmax, sigmay)

#Simplified threshold function, assumes maxVal to be 255
def threshold(image, threshold, thresholdType = cv2.THRESH_BINARY):
    _, threshed = cv2.threshold(image, threshold, 255, thresholdType)
    return threshed

#Simplified edge detection, uses Canny edge detection
def edgeDetect(image, preBlurred = False):
    if image.channels() > 1:
        image = toGray(image)
    if not preBlurred:
        image = blur(image)
    image = cv2.Canny(image, 50, 150)
    return image

#Simplified findContours, assumes RETR_EXTERNAL and CHAIN_APPROX_SIMPLE
def getContours(image):
    contours, _ = cv2.findContours(image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours

#Used on binary images to remove blobs too big to be removed by blurring
def removeSmallWhiteBlobs(image, aggressiveness = 4):
    image = cv2.erode(image, None, iterations = aggressiveness)
    image = cv2.dilate(image, None, iterations = aggressiveness)
    return image

#Used on binary images to remove blobs too big to be removed by blurring
def removeSmallBlackBlobs(image, aggressiveness = 4):
    image = cv2.dilate(image, None, iterations = aggressiveness)
    image = cv2.erode(image, None, iterations = aggressiveness)
    return image

#Counts the approximate number of vertices of a contour
def countVertices(contour):
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.04 * peri, True)
    return len(approx)

#Classifies a contour as a generic polygon shape
def identifyShape(contour, circleLimit = 10):
    nVertices = countVertices(contour)
    shape = 'circle'
    if nVertices > circleLimit:
        return shape
    elif nVertices == 1:
        shape = 'point'
    elif nVertices == 2:
        shape = 'line'
    elif nVertices == 3:
        shape = 'triangle'
    elif nVertices == 4:
        shape = 'quadrilateral'
    elif nVertices == 5:
        shape = 'pentagon'
    elif nVertices == 6:
        shape = 'hexagon'
    elif nVertices == 7:
        shape = 'heptagon'
    elif nVertices == 8:
        shape = 'octagon'
    elif nVertices == 9:
        shape = 'nonagon'
    elif nVertices == 10:
        shape = 'decagon'
    return shape
