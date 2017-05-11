import cv2
from .. import global_constants as gc

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

#Wrapper for Useless
class Useless(Camera):
    def __init__(self):
        super(Bill, self).__init__(gc.USELESS_INDEX, gc.TED_MATRIX, gc.TED_DISTORTION)

    #Useless has not currently been calibrated, so all images are distorted
    def getImage(self):
        return self._getRawImage()
