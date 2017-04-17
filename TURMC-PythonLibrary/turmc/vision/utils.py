import cv2
import sys
from .. import global_constants as gc
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#=============================Simplified Versions of Common Functions=========================

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
    if len(image[0][0]) > 1:
        image = toGray(image)
    contours, _ = cv2.findContours(image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours

#Returns the largest contour from a given set
def getLargestContour(contours):
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

#============================Custom Tools to Make Life Easier=================================

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

#Returns a binary mask highlighting zones matching specified hsv bounds
def getColorThresholdMask(image, hue, hueMargin = 20, satLimit = 100, valLimit = 30, swapBR = False):
    lowerFilter = np.array([hue - hueMargin, satLimit, valLimit], dtype = np.uint8)
    upperFilter = np.array([hue + hueMargin, 255, 255], dtype = np.uint8)
    if swapBR:
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    else:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lowerFilter, upperFilter)
    return mask

#Finds the largest contour matching the specified color
#Current color options are 'green' and 'red'
def findColoredContour(image, color):
    if color == 'green':
        mask = getColorThresholdMask(image, 50, hueMargin = 10, satLimit = 50)
    elif color == 'red':
        mask = getColorThresholdMask(image, 127, hueMargin = 8, satLimit = 120)
    thresholded = cv2.bitwise_and(image, image, mask = mask)
    contour = getLargestContour(getContours(thresholded))
    return contour


#==========================Unit and Data Conversion Tools======================

#Converts sensor_msgs.msg/Image to OpenCV Mat
def ImgMsg2Mat(img_msg):
    return bridge.imgmsg_to_cv2(img_msg, desired_encoding = 'passthrough')

#Converts OpenCV Mat to sensor_msgs.msg/Image
def Mat2ImgMsg(mat):
    return bridge.cv2_to_imgmsg(mat, encoding = 'passthrough')

#Converts a numpy ndarray / cv2 image to a numpy binary string
def Mat2Str(mat):
    height = len(mat)
    width = len(mat[0])
    channels = len(mat[0][0])
    return mat.tostring(), height, width, channels

#Converts a numpy string of ndarray into an ndarray
def Str2Mat(string, height, width, channels = 3, dtype = np.uint8):
    return np.fromstring(string, dtype = dtype).reshape(height, width, channels)

#=========================================Math Tools==========================================

#Given two legs of a triangle and a base, calculates the length of the base's median
#Negative returns indicate errors
def medianFormula( leg_m, leg_n, base):
    #1/2 * sqrt(2*m^2 + 2*n^2 - k^2)
    try:
        return 0.5 * math.sqrt(2 * leg_m ** 2 + 2 * leg_n ** 2 - base ** 2)
    except:
        sys.stderr.write("WARNING: Complex value of median")
        sys.stderr.write("leg_m = " + str(leg_m))
        sys.stderr.write("leg_n = " + str(leg_n))
        sys.stderr.write("base = " + str(base))
        return -1

#==========================================Classes============================================

#Wrapper for cv2's VideoCapture which automatically undistorts the images it captures
class Camera():
    def __init__(self, index, cameraMatrix, distortionConstants):
        self.cam = cv2.VideoCapture(index)
        self.matrix = cameraMatrix
        self.distortion = distortionConstants

    def __del__(self):
        self.cam.release()

    def _getRawImage(self):
        image = self.cam.grab()
        image = self.cam.retrieve(image)
        return image

    def getImage(self):
        image = self._getRawImage()
        image = cv2.undistort(image, self.matrix, self.distortion)
        return image

    def release(self):
        self.cam.release()

#Wrapper for Ted
class Ted(Camera):
    def __init__(self):
        super(Ted, self).__init__(gc.TED_INDEX, gc.TED_MATRIX, gc.TED_DISTORTION)

#Wrapper for Bill
class Bill(Camera):
    def __init__(self):
        super(Bill, self).__init__(gc.BILL_INDEX, gc.BILL_MATRIX, gc.BILL_DISTORTION)
