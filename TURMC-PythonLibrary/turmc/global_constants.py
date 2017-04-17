#!/usr/bin/env python

import numpy as np
import pigpio

#=============Measurements==============

#This is the real-world distance between the two cameras. It is in cm and typically used as 'k'
CAMERA_AXIAL_DISTANCE= 1.0

#This is the real-world distance between the centers of the two beacons. It is in cm and typically used as 'K'
BEACON_CENTER_DISTANCE = 1.0

#=============Hardware==============

#These correspond to Ubuntu's /dev/video# numbers. We should use udev rules to set static assignment.
STAR_CAMERA_INDEX = 0
PORT_CAMERA_INDEX = 1
TED_INDEX = 0
BILL_INDEX = 1

PIGPIO_PI_REFERENCE = pigpio.pi()

#=============Networking==============

#The IP address of the Pi
RASPBERRY_PI_IP = ''

#The IP address of the Nuc
NUC_IP = ''

#The IP address of the base computer
BASE_COMPUTER_IP = ''

#The IP address (and port?) of the ROS Master Node.
ROS_MASTER_URI = ''

#=============Camera Calibration Data================
#Calibrated using 600 images each, March 3 2017, Sam Wilson

bfx = 379.7279
bfy = 380.5608
bcx = 292.7610
bcy = 226.6971
br1 = -0.3290
br2 = 0.1545
br3 = -0.0413
BILL_MATRIX = np.matrix([ [bfx, 0, bcx], [0, bfy, bcy], [0, 0, 1] ])
BILL_DISTORTION = np.matrix([ br1, br2, 0, 0, br3])

tfx = 373.7896
tfy = 374.5872
tcx = 317.5320
tcy = 237.4582
tr1 = -0.3321
tr2 = 0.1574
tr3 = -0.04267
TED_MATRIX = np.matrix([ [tfx, 0, tcx], [0, tfy, tcy], [0, 0, 1] ])
TED_DISTORTION = np.matrix([ tr1, tr2, 0, 0, tr3])
