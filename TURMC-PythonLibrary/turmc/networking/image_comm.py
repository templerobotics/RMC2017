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
        self.isListening = False

    #Closes the server
    def __del__(self):
        self._server.close()
        self.isListening = False

    #Receives an image and sends it to the callback function
    def _receiveImage(self, metadata, metaaddress):
        #Attempts to parse the metadata
        try:
            metadata = json.loads(metadata)
        except JSONDecodeError:
            print("Failed to decode metadata")
            return
        #Creates a filepath out of the given name
        filepath = metadata['name'] + '.jpg'
        receivedPackets = 0
        #Opens a file to write the received data
        with open(filepath, 'wb+') as handle:
            #Loop to keep receiving packets
            while True:
                #Receives data; if the timeout expires we save what we have and return to the server loop
                try:
                    self._server.settimeout(2)
                    data, address = self._server.recvfrom(self.bufferSize)
                except timeout:
                    break
                #Ignores packets from different addresses. This shouldn't be necessary, but it's more secure
                if address == metaaddress:
                    #A packet of the string 'end' will immediately finish the image
                    if data == 'end':
                        break
                    else:
                        #Writes the received bytes to the file
                        handle.write(data)
                        receivedPackets += 1
        #Loads the file that was downloaded
        image = cv2.imread(filepath)
        #Sends the image and its name to the callback function
        self._callback(metadata['name'], image)

    #Listens for an incoming image signal
    def _listen(self):
        #Waits for a packet
        data, address = self._server.recvfrom(self.bufferSize)
        #The string 'start' indicates that an image is about to be sent
        if data == 'start':
            #Attempts to receive metadata about the upcoming image.
            #If metadata is not received within the timeout, we go back to the server loop
            try:
                self._server.settimeout(2)
                metadata, metaaddress = self._server.recvfrom(self.bufferSize)
                #We make sure the metadata we received is from the correct address
                while (metaaddress != address):
                    metadata, metaaddress = self._server.recvfrom(self.bufferSize)
            except timeout:
                return
            #With metadata received, move on to receiving the actual image
            self._receiveImage(metadata, metaaddress)

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
        #Prepares metadata
        metadata['name'] = name
        metadata['size'] = os.path.getsize('temp.jpg')
        metadata['packetcount'] = ceil(metadata['size'] / self.bufferSize)
        #Sends the 'start' signal
        self._client.sendto('start', self._address)
        #Sends the metadata as a JSON encoded string
        self._client.sendto(json.dumps(metadata), self._address)
        #Opens the file for sending
        with open('temp.jpg', 'rb') as handle:
            data = handle.read(self.bufferSize)
            while data:
                if self._client.sendto(data, self._address):
                    data = handle.read(self.bufferSize)
        self._client.sendto('end', self._address)
