#!/usr/bin/env python

from socket import *
import MotorControl as mc

def toDict(string):
    dict = {}
    pairs = string.split(';')
    del pairs[-1]
    
    for word in pairs:
        keyValue = word.split(':')
        dict[keyValue[0]] = float(keyValue[1])
    return dict

rm = mc.SabretoothDCMotor(6)
lm = mc.SabretoothDCMotor(12)
robot = mc.RobotDrivetrain(rm, lm)

server = socket(AF_INET, SOCK_DGRAM)
server.bind(('', 5555))

while True:
    message, addr = server.recvfrom(2048)
    data = toDict(message)
    #This configuration makes the right side behavior work correctly; untested for the left side
    robot.arcadeDrive(data['joyX'], data['joyY'], invertY = True, swapRL = True)
    print('if you see this, things are probably working. no guarantees')
