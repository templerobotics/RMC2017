#!/usr/bin/env python

from serial import *
import RPi.GPIO as GPIO

class MultiChannelSerial:

    def __init__(port, switchboardPins, baudrate = 9600, timeout = 1):

        self.serialCom = serial.Serial(port, baudrate = baudrate, timeout = timeout)

        self.pins = switchboardPins

        if len(self.pins) == 0:
            print("ERROR: No pins were supplied to the MultiChannelSerial object!")
            return False

        self.NUM_CHANNELS = 2 ** len(self.pins)

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pins, GPIO.OUT)

        self.current_channel = 1
        self._setChannel(0)

        return True

    def _setChannel(channel):

        if channel == self.current_channel:
            return True
        elif channel >= self.NUM_CHANNELS:
            return False

        #Converts the channel value into a reversed bit string representing pin states
        bits = '{0:0{count}b}'.format(channel, count = len(self.pins))[::-1]

        #Sets the appropriate outputs of the pins
        #Order of the pins is important
        for i in range(len(self.pins)):
            if bits[i] == '1':
                GPIO.output(self.pins[i], GPIO.HIGH)
            elif bits[i] == '0':
                GPIO.output(self.pins[i], GPIO.LOW)

        self.current_channel = channel

        return True

    def write(data, channel):

        if self._setChannel(channel):
            return self.serialCom.write(data)

        return None

    def read(channel):

        if self._setChannel(channel):
            return self.serialCom.read()

        return None

    def readline(channel):

        if self._setChannel(channel):
            return self.serialCom.readline()

        return None
