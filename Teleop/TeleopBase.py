import pygame
from time import sleep
from turmc.networking.joystick import Client
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
    data["joyX"] = j.get_axis(0)
    data["joyY"] = j.get_axis(1)
    data["throttle"] = j.get_axis(2)
    data["button1"] = j.get_button(0)
    data["button2"] = j.get_button(1)
    data["button3"] = j.get_button(2)
    data["button4"] = j.get_button(3)
    data["button5"] = j.get_button(4)
    data["button6"] = j.get_button(5)
    data["button7"] = j.get_button(6)
    data["button8"] = j.get_button(7)
    data["button9"] = j.get_button(8)
    data["button10"] = j.get_button(9)
    data["button11"] = j.get_button(10)
    data["button12"] = j.get_button(11)
    data["hatX"], data["hatY"] = j.get_hat(0)

#Run if this script is run as main
def main():

    #Defines the socket objects of the RPi and the Nuc; used for sending them data
    RPi = Client(RASPBERRY_PI_IP, RASPBERRY_PI_PORT)
    Nuc = Client(NUC_IP, NUC_PORT)

    #This is intended to continue until it is manually stopped
    while True:
        #The try-except block is to correctly shutdown the socket and pygame
        try:
            grabJoystickData()
            RPi.send(data)
            Nuc.send(data)
            sleep(1.0 / rate)
        except KeyboardInterrupt:
            del RPi, Nuc
            pygame.quit()
            break

#Boilerplate
if __name__ == '__main__':
    main()
