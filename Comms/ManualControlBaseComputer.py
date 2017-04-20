import pygame
from time import sleep
from socket import *


#Pi Information
targetIP = '192.168.1.101'
port = 5555

#Nuc Info
Nuc_targetip = '192.168.1.102'
Nuc_port = 15000

pygame.joystick.init()
pygame.init()

j = pygame.joystick.Joystick(0)
j.init()

clientSocket = socket(AF_INET, SOCK_DGRAM)
pyAddress = (targetIP, port)
nucAddress =(Nuc_targetip, Nuc_port)

data = {}
def toString(dictionary):
    string = ''
    typeCode = ''
    for key in dictionary:
        valueType = type(dictionary[key])

        if valueType is int:
            typeCode = 'i'
        elif valueType is float:
            typeCode = 'f'
        elif valueType is long:
            typeCode = 'l'
        elif valueType is bool:
            typeCode = 'b'
        elif valueType is None:
            typeCode = 'n'
        else:
            typeCode = 's'

        string += '{}:{}:{};'.format(key, typeCode, str(dictionary[key]))

    return string

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

while True:
    try:
        grabJoystickData()
        message = toString(data)
        print(message)
        clientSocket.sendto(message, pyAddress)
        clientSocket.sendto(message, nucAddress)
        sleep(0.3)
    #Trying to get it to saftely exit...
    except (KeyboardInterrupt):
        clientSocket.shutdown()
        clientSocket.close()
        pygame.quit()
        break
