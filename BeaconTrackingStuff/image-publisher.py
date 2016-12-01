#!/usr/bin/env python

import rospy
from cv2 import VideoCapture
import vision-tools as vt
from sensor_msgs.msg import Image
#from custom_msgs.msg import DualImage

star_camera_index = 0
port_camera_index = 1

sCam = VideoCapture(star_camera_index)
pCam = VideoCapture(port_camera_index)

pub = rospy.Publisher('/images', DualImage, queue_size = 10)

def captureImages():
    sImage = sCam.grab()
    pImage = pCam.grab()

    sImage, _ = sCam.retrieve(sImage)
    pImage, _ = pCam.retrieve(pImage)

    sMsg = vt.Mat2ImgMsg(sImage)
    pMsg = vt.Mat2ImgMsg(pImage)

    message = DualImage()

    message.sImage = sMsg
    message.pImage = pMsg

    pub.publish(message)

def cleanup():
    sCam.release()
    pCam.release()

def listener():
    rospy.init_node('image-publisher', anonymous = True)

    rospy.on_shutdown(cleanup)

    rospy.Subscriber('/images', String, captureImages)

    rospy.spin()

if __name__ == '__main__':
    listener()
