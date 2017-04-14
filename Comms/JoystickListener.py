#!/usr/bin/env python2

from socket import *

#Converts a string of the form 'key1:type1:value1;key2:type2:value2;' to a dictionary of string keys and float values
def toDict(string):
    dictionary = {}
    pairs = string.split(';')
    del pairs[-1]
    for word in pairs:
        keyValue = word.split(':')
	typeCode = keyValue[1]
	if typeCode == 'i':
	    dictionary[keyValue[0]] = int(keyValue[2])
	elif typeCode == 'f':
            dictionary[keyValue[0]] = float(keyValue[2])
        elif typeCode == 'l':
            dictionary[keyValue[0]] = long(keyValue[2])
        elif typeCode == 'b':
            dictionary[keyValue[0]] = bool(keyValue[2])
        elif typeCode == 'n':
            dictionary[keyValue[0]] = None
        else:
            dictionary[keyValue[0]] = keyValue[2]
    return dictionary

class JoystickListener():
    def __init__(self, port, eventHandlerCallback):
        self._server = socket(AF_INET, SOCK_DGRAM)
        self._server.bind(('', port))
        self.callback = eventHandlerCallback
        self.listen = False

    def __del__(self):
        self.listen = False

    def startListening(self):
        self.listen = True
        while self.listen:
            message, addr = self._server.recvfrom(2048)
            data = toDict(message)
            self.callback(data)

    def stopListening(self):
        self.listen = False
