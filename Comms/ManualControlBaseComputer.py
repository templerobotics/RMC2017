import pygame
from time import sleep
from socket import *
targetIP = '192.168.1.100'
port = 5555

pygame.joystick.init()
pygame.init()

j = pygame.joystick.Joystick(0)
j.init()

clientSocket = socket(AF_INET, SOCK_DGRAM)
serverAddress = (targetIP, port)

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

    data["leftX"] = j.get_axis(0)
    data["leftY"] = j.get_axis(1)
    data["triggers"] = j.get_axis(2)
    data["rightY"] = j.get_axis(3)
    data["rightX"] = j.get_axis(4)
    #data["button1"] = j.get_button(0)
    #data["button2"] = j.get_button(1)
    #data["button3"] = j.get_button(2)
    #data["button4"] = j.get_button(3)
    #data["button5"] = j.get_button(4)
    #data["button6"] = j.get_button(5)
    #data["button7"] = j.get_button(6)
    #data["button8"] = j.get_button(7)
    #data["button9"] = j.get_button(8)
    #data["button10"] = j.get_button(9)
    #data["button11"] = j.get_button(10)
    #data["button12"] = j.get_button(11)

while True:
    try:
        grabJoystickData()
        message = toString(data)
        print(message)
        clientSocket.sendto(message, serverAddress)
        sleep(0.3)
    #Trying to get it to saftely exit...
    except (KeyboardInterrupt):
        clientSocket.shutdown()
        clientSocket.close()
        pygame.quit()
        pygame.QUIT()
        break


