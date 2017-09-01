import pygame
from time import sleep
from turmc.networking.joystick import JSONClient
from turmc.global_constants import *

#Rate is the number of updates to send per second
rate = 2

#Initialize pygame
pygame.init()
pygame.joystick.init()

#Initialize the joystick
j = pygame.joystick.Joystick(0)
j.init()

#A dictionary that will hold all the joystick data
data = {}

#Updates the joystick dictionary with the current joystick values
def grabJoystickData():
    pygame.event.get()
    data["x"] = j.get_axis(0) * 0.9
    data["y"] = j.get_axis(1) * 0.9

    if j.get_button(2) == 1.0:
        data['bucket'] = -1.0
    elif j.get_button(4) == 1.0:
        data['bucket'] = 1.0
    else:
        data['bucket'] = 0.0

    if j.get_button(3) == 1.0:
        data['arm'] = -1.0
    elif j.get_button(5) == 1.0:
        data['arm'] = 1.0
    else:
        data['arm'] = 0.0

#Run if this script is run as main
def main():

    #Defines the socket objects of the RPi and the Nuc; used for sending them data
    RPi = JSONClient('192.168.42.1', 27000)

    #This is intended to continue until it is manually stopped
    while True:
        #The try-except block is to correctly shutdown the socket and pygame
        try:
            grabJoystickData()
            RPi.send(data)
            sleep(1.0 / rate)
        except KeyboardInterrupt:
            pygame.quit()
            break

#Boilerplate
if __name__ == '__main__':
    main()
