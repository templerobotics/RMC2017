#!/usr/bin/env python

#Sam Wilson, 4/7/2017

import serial

#List of commands for packetized serial mode of a Sabertooth Motor Controller
motorCommand = {'M1:Forward': 0,
                'M1:Backward': 1,
                'MinVoltage': 2,
                'MaxVoltage': 3,
                'M2:Forward': 4,
                'M2:Backward': 5,
                'M1:Bidirectional': 6,
                'M2:Bidirectional': 7,
                'MM:Forward': 8,
                'MM:Backward': 9,
                'MM:TurnRight': 10,
                'MM:TurnLeft': 11,
                'MM:BidirectionalMove': 12,
                'MM:BidirectionalTurn': 13
                }

#Calculates checksum that the Sabertooth will be expecting
def checksum(address, command, data):
    return (address + command + data) & 0b01111111

#Creates a serial interface designed to send packets to Sabertooth Motor Controllers
class SPSI:

    def __init__(port):

        #Serial output
        self.com = serial.Serial(port, baudrate = 9600, timeout = 1)

        #Controller warmup before baudrate configuration
        sleep(2)

        #Baudrate configuration
        self.com.write(170)

    #Sends the packet along with a checksum
    def sendPacket(address, command, data):
        self.com.write(address)
        self.com.write(command)
        self.com.write(data)
        self.com.write(checksum(address, command, data))
