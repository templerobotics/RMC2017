from socket import *

def toString(dictionary):
    string = ''
    for key in dictionary:
        string += '{}:{};'.format(key, str(dictionary[key]))
    return string

def toDict(string):
    dictionary = {}
    pairs = string.split(';')
    del pairs[-1]
    for word in pairs:
        keyValue = word.split(':')
        dictionary[keyValue[0]] = float(keyValue[1])
    return dictionary

class UDPSender:
    def __init__(self, targetIP = '127.0.0.1', port = 5555):
        self._socket = socket(AF_INET, SOCK_DGRAM)
        self._targetAddress = (targetIP, port)

        return self._targetAddress

    def sendString(self, string):
        return self._socket.sendto(string, self._targetAddress)

    def sendDictionary(self, dictionary):
        string = toString(dictionary)
        return self.sendString(string)

class UDPReceiver:
    def __init__(self, port = 5555, IP = '', bufferSize = 2048):
        self._socket = socket(AF_INET, SOCK_DGRAM)
        self._targetAddress = (IP, port)
        self._bufferSize = bufferSize

        self._socket.bind(self._targetAddress)

        return self._targetAddress

    def listenForString(self):
        string, address = self._socket.recvfrom(self._bufferSize)
        return string, address

    def listenForDictionary(self):
        string, address = self.listenForString()
        return toDict(string), address
