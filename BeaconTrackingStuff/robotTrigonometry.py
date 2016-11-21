#! python2

import math
import cv2

#IMPORTANT
#This variable has to be set to the distance between the yaw axes of our two cameras
#Many of our calculations require this value, so it is important that it != null
#We should probably have this as a global ROS variable, because of its importance
k = 1.0
#These are used in calculating camera angles
cameraCenterX = 319.5 # (camera pixel width - 1) / 2
cameraCenterY = 119.5 # (camera pixel height - 1) / 2
focalLengthP = 555 # (camera pixel width - 2) / ( 2 * tan( FOV / 2) )
#focalLengthYP = 555 # (camera pixel height - 1) / ( 2 * tan( FOV / 2) )

#Sam Wilson 10/16/16
def getRobotCoords(leftAngle, rightAngle, k = k):
    #Determines the robots coordinates in an xy plane with the beacon at the origin.
    #Everything in the arena has a positive x coordinate, with the mining area being at the "right" end

    #Convert angles to radians for the trig functions
    theta1 = math.radians(leftAngle)
    theta2 = math.radians(rightAngle)

    #Calculate the unknown two sides of our positioning triangle, m and n
    m = (k * math.sin(theta1)) / math.sin(theta1 + theta2)
    n = (k * math.sin(theta2)) / math.sin(theta1 + theta2)

    #Calculate the distance from the center of our robot to the beacon
    #This is the median of our positioning triangle
    p = 0.5 * math.sqrt(2*m*m + 2*n*n - k*k)

    #Calculate the distance from the beacon to the robot's x coordinate
    #This is the altitude of our positioning triangle
    #If we consider the beacon the origin of a cartesian plane, then this
    #value is our robot's x-coordinate
    j = (k * math.sin(theta1) * math.sin(theta2)) / math.sin(theta1 + theta2)

    #Calculate the distances from the cameras to the intersection of j and k
    #These ones are more complicated to describe, but they are necessary to
    #determine if the y coordinate should be negative or positive. See Sam Wilson
    #for an in depth explanation
    q = (k * math.cos(theta1) * math.sin(theta2)) / math.sin(theta1 + theta2)
    r = (k * math.sin(theta1) * math.cos(theta2)) / math.sin(theta1 + theta2)

    #Calculate s, the distance between p and j on k. This equates to the robot's
    #y coordinate. This value will always be greater than or equal to zero, which
    #is why we need q and r to determine the actual sign
    s = math.sqrt(p*p - j*j)

    #Determine the sign of the y coordinate based on q and r
    if q > r:
        s = -s

    #Now that everything has been calculated, return the values as an ordered pair
    return {'x': j, 'y': s}

#Sam Wilson 10/16/16
def getObjectCoords(leftAngle, rightAngle, robotCoords, k = k):
    #Returns the global coordinates of an object that the cameras have focused on.
    #Requires the robot to know its current coordinates

    #Convert angles to radians for the trig functions
    theta1 = math.radians(leftAngle)
    theta2 = math.radians(rightAngle)

    #Calculate the unknown two sides of our positioning triangle, m and n
    m = (k * math.sin(theta1)) / math.sin(theta1 + theta2)
    n = (k * math.sin(theta2)) / math.sin(theta1 + theta2)

    #Calculate the distance from the center of our robot to the beacon
    #This is the median of our positioning triangle
    p = 0.5 * math.sqrt(2*m*m + 2*n*n - k*k)

    #Calculate the distance from the object to the robot's x coordinate
    #This is the x offset from our robot to the object
    j = (k * math.sin(theta1) * math.sin(theta2)) / math.sin(theta1 + theta2)

    #Calculate the distances from the cameras to the intersection of j and k
    #These ones are more complicated to describe, but they are necessary to
    #determine if the y coordinate should be negative or positive. See Sam Wilson
    #for an in depth explanation
    q = (k * math.cos(theta1) * math.sin(theta2)) / math.sin(theta1 + theta2)
    r = (k * math.sin(theta1) * math.cos(theta2)) / math.sin(theta1 + theta2)

    #Calculate s, the distance between p and j on k. This is the y offset between
    #the object and the robot. This value will always be greater than or equal to
    #zero, which is why we need q and r to determine the actual sign
    s = math.sqrt(p*p - j*j)

    #Determine the sign of the y coordinate based on q and r
    if q > r:
        s = -s

    #Calculate the objects global coordinates based on the offsets and the robot coordinates
    objectx = robotCoords['x'] + j
    objecty = robotCoords['y'] + s

    #Return the coordinates
    return {'x': objectx, 'y': objecty}

#Sam Wilson 10/16/16
def getCameraAngles():
    #Ignore this for now, it's for a later implementation of the code and won't be finished till
    #we're running a ROS environment and have the cameras and servos. I'm just writing the code
    #here for when we need it later. Also, this function will likely be moved to a vision interface node

    #The math for this assumes that 0 degrees is to the right of the robot, 90 is forward, 180 is left, and
    #270 is back

    #Most of these fields should be self-explanatory. The 'usable' field is used for error handling. Rather
    #than actually calling an error, we just assume the function caller (AKA us) will check the 'usable'
    #field before attempting to use the data.
    cameraAngles = {'leftAngle': 0.0,
                    'rightAngle': 0.0,
                    'facingForwards': False,
                    'usable': True }

    #These camera angles will be the ~0-360 degree angles from the servos. "getRawAngle" may be a call
    #to the camera controllers, servo controllers, or perhaps a vision hardware interface node.
    #cam1Angle = getRawAngle('cam1')
    #cam2Angle = getRawAngle('cam2')

    #If the cameras are parallel, the distance to the object they're "focused" on is infinity, which
    #doesn't make sense in a 7x3 meter arena.
    if cam1Angle == cam2Angle:
        cameraAngles['usable'] = False
        return cameraAngles

    #Checks if the cameras are facing the same direction (forward or backward). If they aren't, they can't
    #be looking at the same thing
    if (cam1Angle > 180 and cam1Angle < 360) and (cam2Angle > 180 and cam2Angle < 360):
        cameraAngles['facingForwards'] = False
    elif (cam1Angle < 180 and cam1Angle > 0) and (cam2Angle < 180 and cam2Angle > 0):
        cameraAngles['facingForwards'] = True
    else:
        cameraAngles['usable'] = False
        return cameraAngles

    #Determines which camera is on the right and which one is on the left
    if cameraAngles['facingForwards']:
        cameraAngles['rightAngle'] = cam1Angle
        cameraAngles['leftAngle'] = cam2Angle
    else:
        #These angles are in the 3rd or 4th quadrant, so subtracting 180 will bring them into the 1st or 2nd
        cameraAngles['rightAngle'] = cam1Angle - 180.0
        cameraAngles['leftAngle'] = cam2Angle - 180.0

    cameraAngles['rightAngle'] = 180 - cameraAngles['rightAngle']
    
    return cameraAngles

#Sam Wilson 11/21/2016
def calculateAdjustedAngles(servoYawAngle, servoPitchAngle, xCoord, yCoord):
    #This function is the core of beacon tracking. It calculates the angle the
    #camera would have to turn to face an object it detects. When combined with
    #the servo angle, it gives us an accurate estimate of what direction the
    #servo would be at to be facing the object. Very important for other formulas.

    #Calculates offset. Requires camerCenters and focal length to be calculated.
    #Sam has the formulas to calculate those things.
    yawOffset = math.arctan((xCoord - cameraCenterX) / focalLengthP)
    pitchOffset = math.arctan((yCoord - cameraCenterY) / focalLengthP)

    #The signs on these lines will depend on the positive and negative directions
    #of the servos. Once the hardware is in place, final testing can be done.
    adjustedYaw = servoYawAngle - yawOffset
    adjustedPitch = servoPitchAngle - pitchOffset

    #Stores the values in a dictionary so we can access values via common names.
    adjustedAngles = { 'yaw': adustedYaw,
                       'pitch': adjustedPitch }

    return adjustedAngles
