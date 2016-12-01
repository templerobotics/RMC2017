#!/usr/bin/env python

#=============Measurements==============

#This is the real-world distance between the two cameras. It is in cm and typically used as 'k'
CAMERA_AXIAL_DISTANCE= 1.0

#This is the real-world distance between the centers of the two beacons. It is in cm and typically used as 'K'
BEACON_CENTER_DISTANCE = 1.0

#=============Hardware==============

#These correspond to Ubuntu's /dev/video# numbers. We should use udev rules to set static assignment.
STAR_CAMERA_INDEX = 0
PORT_CAMERA_INDEX = 1

#=============Networking==============

#The IP address of the Pi
RASPBERRY_PI_IP = ''

#The IP address of the Nuc
NUC_IP = ''

#The IP address of the base computer
BASE_COMPUTER_IP = ''

#The IP address (and port?) of the ROS Master Node.
ROS_MASTER_URI = ''
