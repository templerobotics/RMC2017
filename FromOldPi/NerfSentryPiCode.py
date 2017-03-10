from socket import *
import pigpio
import MotorControl as MC

PI = pigpio.pi()

PAN_PIN = 12
FIRING_PIN = 6

panMotor = MC.VexMotorModule(PAN_PIN)

def stopShooting():
    PI.write(FIRING_PIN, pigpio.LOW)

def startShooting():
    PI.write(FIRING_PIN, pigpio.HIGH)

def turnRight():
    panMotor.setSpeed(-0.75)

def turnLeft():
    panMotor.setSpeed(0.75)

def stopTurning():
    panMotor.setSpeed(0.0)

serverSocket = socket(AF_INET, SOCK_DGRAM)
serverSocket.bind(('', 12000))

while True:
    message, address = serverSocket.recvfrom(1024)

    if message == 'right':
        stopShooting()
        turnRight()
    elif message == 'left':
        stopShooting()
        turnLeft()
    elif message == 'stopTurning':
        stopTurning()
    elif message == 'shoot':
        stopTurning()
        startShooting()
    elif message == 'stopShooting':
        stopShooting()

    print(message)
