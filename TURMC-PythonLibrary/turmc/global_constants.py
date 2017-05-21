#!/usr/bin/env python

import numpy as np
pigpioLoaded = True
try:
    import pigpio
except ImportError:
    print("Pigpio not found; skipping")
    pigpioLoaded = False
#=============Measurements==============

#This is the real-world distance between the two cameras. It is in cm and typically used as 'k'
CAMERA_AXIAL_DISTANCE= 1.0

#This is the real-world distance between the centers of the two beacons. It is in cm and typically used as 'K'
BEACON_CENTER_DISTANCE = 1.0

CALIBRATION_STICKER_ANGLE_OFFSET = 0
DISTANCE_ROBOT_CENTER_TO_CAMERA_STEPPER = 0

#=============Hardware==============

#These correspond to Ubuntu's /dev/video# numbers. We should use udev rules to set static assignment.
TED_INDEX = 0
BILL_INDEX = 1
USELESS_INDEX = 2

#These are the RPi GPIO pin numbers of the drill stepper control wires
DRILL_STEPPER_PUL_PIN = 20
DRILL_STEPPER_DIR_PIN = 21

#The Phidget controller index for Ted's Stepper
TED_STEPPER_INDEX = 0

#Global reference to Pigpio's Pi handle. Used by anything using Pigpio
PIGPIO_PI_REFERENCE = pigpio.pi() if pigpioLoaded else None

#=============Networking==============

#The IP address of the Pi
RASPBERRY_PI_IP = ''
RASPBERRY_PI_PORT = 12002

#The IP address of the Nuc
NUC_IP = ''
NUC_PORT = 12001

#The IP address of the base computer
BASE_COMPUTER_IP = ''
BASE_COMPUTER_PORT = 12000

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

#=============Serial Addresses================

SERIAL_ADDR_DRIVETRAIN = 129
SERIAL_ADDR_LINEAR_ACUATORS = 130
SERIAL_ADDR_OTHER_MOTORS = 128

MOTOR_NUMBER_DRILL_ACTUATOR = 2
MOTOR_NUMBER_CONVEYOR_ACTUATOR = 1
MOTOR_NUMBER_AUGER = 2
MOTOR_NUMBER_AUGUR = MOTOR_NUMBER_AUGER
MOTOR_NUMBER_CONVEYOR = 1

#================ROS Related Settings=================

#The frequency, in Hz, that Bill and Useless publish images to the ROS network
IMAGE_PUBLISH_FREQUENCY = 1

#ROS Topics
TOPIC_TED_IMAGES = 'images/Ted'
TOPIC_BILL_IMAGES = 'images/Bill'
TOPIC_USELESS_IMAGES = 'images/Useless'
TOPIC_TED_STEPPER = 'hardware/Ted'
TOPIC_BEACON_TRACKING = 'data/beacontracking'
TOPIC_MOTOR_CONTROL = 'hardware/Pi'
TOPIC_ENCODER = 'data/encoder'
TOPIC_MAPPING = 'control/mapping'
TOPIC_PATHFINDING = 'data/pathfinding'
TOPIC_IMU = 'data/imu'
TOPIC_ODOM = ''
TOPIC_ODOM_FILTERED = ''
TOPIC_MINING_STATE = 'state/mining'
TOPIC_MINING_CONTROL = 'control/mining'
TOPIC_MINING_REDIRECT = 'redirect/mining'
TOPIC_DUMPING_STATE = 'state/dumping'
TOPIC_DUMPING_CONTROL = 'control/dumping'
TOPIC_DUMPING_REDIRECT = 'redirect/dumping'
TOPIC_NAVIGATING_STATE = 'state/navigating'
TOPIC_NAVIGATING_CONTROL = 'control/navigating'
TOPIC_NAVIGATING_REDIRECT = 'redirect/navigating'
