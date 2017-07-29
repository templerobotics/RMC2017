import cv2
from .. import global_constants as gc
import urllib2
import time
import numpy as np

IP_CAMERA_COMMANDS = {'STOP' = 1,
                      'UP' = 0,
                      'DOWN' = 2,
                      'RIGHT' = 6,
                      'LEFT' = 4,
                      }

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

class EasyN_IPCamera:

    def __init__(self, ipaddress, username = 'admin', password = ''):
        self.address = ipaddress
        self.username = username
        self.password = password

        pwd_mgr = urllib2.HTTPPasswordMgrWithDefaultRealm()
        pwd_mgr.add_password(None, 'http://{}'.format(self.address), self.username, self.password)
        handler = urllib2.HTTPBasicAuthHandler(pwd_mgr)

        self.conn = urllib2.build_opener(handler)

        self._url_snapshot = 'http://{address}/snapshot.cgi'.format(address = self.address)
        self._url_command = 'http://{address}/decoder_control.cgi?username={username}&password={password}&command='.format(address = self.address, username = self.username, password = self.password) + '{command}'

    def read(self):
        try:
            response = self.conn.open(self._url_snapshot)
        except urllib2.HTTPError as e:
            raise e
        except urllib2.URLError as e:
            raise e
        except Exception as e:
            raise e

        raw_img = np.asarray(bytearray(response.read()), dtype = np.uint8)
        return cv2.imdecode(raw_img, cv2.IMREAD_COLOR)

    def _move(self, command, timeout = None):
        try:
            _ = self.conn.open(self._url_command.format(command = command))
        except urllib2.HTTPError as e:
            raise e
        except urllib2.URLError as e:
            raise e
        except Exception as e:
            raise e

        if timeout is not None:
            time.sleep(timeout)
            self.stop()

    def stop(self):
        self._move(IP_CAMERA_COMMANDS['STOP'])

    def panUp(self, timeout = None):
        self._move(IP_CAMERA_COMMANDS['UP'], timeout)

    def panDown(self, timeout = None):
        self._move(IP_CAMERA_COMMANDS['DOWN'], timeout)

    def panRight(self, timeout = None):
        self._move(IP_CAMERA_COMMANDS['RIGHT'], timeout)

    def panLeft(self, timeout = None):
        self._move(IP_CAMERA_COMMANDS['LEFT'], timeout)

def init():
    global Ted, Bill, Useless
    Ted = Camera(gc.TED_INDEX, gc.TED_MATRIX, gc.TED_DISTORTION)
    Bill = Camera(gc.BILL_INDEX, gc.BILL_MATRIX, gc.BILL_DISTORTION)
    Useless = Camera(gc.USELESS_INDEX, gc.TED_MATRIX, gc.TED_DISTORTION)

if __name__ == 'cameras':
    init()
