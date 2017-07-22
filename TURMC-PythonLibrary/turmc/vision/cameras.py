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
        _, image = self.cam.read()
        return image

    def getImage(self):
        image = self._getRawImage()
        image = cv2.undistort(image, self.matrix, self.distortion)
        return image

    def release(self):
        self.cam.release()

def init():
    global Ted, Bill, Useless
    Ted = Camera(gc.TED_INDEX, gc.TED_MATRIX, gc.TED_DISTORTION)
    Bill = Camera(gc.BILL_INDEX, gc.BILL_MATRIX, gc.BILL_DISTORTION)
    Useless = Camera(gc.USELESS_INDEX, gc.TED_MATRIX, gc.TED_DISTORTION)

if __name__ == 'cameras':
    init()
