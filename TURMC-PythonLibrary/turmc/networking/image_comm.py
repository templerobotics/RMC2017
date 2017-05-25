from socket import *
import cv2
import json
from math import ceil
import os.path

class ImageServer:

    def __init__(self, port, callback, bufferSize = 2048):
        self._port = port
        self._callback = callback
        self.bufferSize = bufferSize

        self._server = socket(AF_INET, SOCK_DGRAM)
        self._server.bind(('', self._port))

        self._server.settimeout(3)

        self.isListening = False

    #Closes the server
    def __del__(self):
        self._server.close()
        self.isListening = False

    #Receives an image and sends it to the callback function
    def _receiveImage(self, sourceAddress):
        #Opens a file to write the received data
        with open('temp.jpg', 'wb+') as handle:
            #Loop to keep receiving packets
            while True:
                #Receives data; if the timeout expires we save what we have and return to the server loop
                try:
                    data, address = self._server.recvfrom(self.bufferSize)
                except timeout:
                    break
                #Ignores packets from different addresses. This shouldn't be necessary, but it's more secure
                if address == sourceAddress:
                    #A packet of the string 'end' will immediately finish the image
                    if data == 'end':
                        break
                    else:
                        #Writes the received bytes to the file
                        handle.write(data)
        #Loads the file that was downloaded
        image = cv2.imread(filepath)
        #Sends the image and its name to the callback function
        self._callback(image)

    #Listens for an incoming image signal
    def _listen(self):
        #Waits for a packet
        try:
            data, address = self._server.recvfrom(self.bufferSize)
        except timeout:
            return
        #The string 'start' indicates that an image is about to be sent
        if data == 'start':
            self._receiveImage(address)

    def start(self):
        self.isListening = True
        while self.isListening:
            self._listen()

    def stop(self):
        self.isListening = False

#Creates a socket able to send an image to an ImageServer
class ImageSender:

    #Requires the ip of the target and the port to use
    def __init__(self, ip, port, bufferSize = 2048):
        self.bufferSize = bufferSize
        self._address = (ip, port)
        self._client = socket(AF_INET, SOCK_DGRAM)

    #Safely disposes of the socket
    def __del__(self):
        self._client.close()

    def sendImage(self, image, name, metadata = {}):
        #Writes the image to a temporary file. This is where the image compression occurs
        cv2.imwrite('temp.jpg', image)
        #Sends the 'start' signal
        self._client.sendto('start', self._address)
        #Opens the file for sending
        with open('temp.jpg', 'rb') as handle:
            data = handle.read(self.bufferSize)
            while data:
                if self._client.sendto(data, self._address):
                    data = handle.read(self.bufferSize)
        self._client.sendto('end', self._address)
