from socket import *
from ..tools.textutils import String2Dictionary, Dictionary2String
from time import sleep
import json

#Creates a Server object for an event based socket system
class Server():

    #Given a port, waits for messages on that channel. Received messages are passed to an event handler
    def __init__(self, port, eventHandlerCallback, bufferSize = 2048, errorCallback = None):
        self._port = port
        self.bufferSize = bufferSize
        self._server = socket(AF_INET, SOCK_DGRAM)
        self._server.bind(('', self._port))
        self.callback = eventHandlerCallback
        self.errorCallback = errorCallback
        self.isBlocking = True
        self.listen = False

    #Stops listening on delete in an attempt to prevent hanging
    def __del__(self):
        self._server.close()
        self.listen = False

    #Configures
    def setBlocking(self, boolean = True):
        if boolean:
            self._server.setblocking(1)
            self.isBlocking = True
        else:
            self._server.setblocking(0)
            self.isBlocking = False

    #Starts an event loop that passes received data back to an event handler
    def start(self):
        self.listen = True
        while self.listen:
            try:
                message, addr = self._server.recvfrom(2048)
                data = String2Dictionary(message)
                self.callback(data)
            except:
                if not self.errorCallback is None:
                    self.errorCallback()
            if not self.isBlocking:
                sleep(0.01)

    #Flags the server to stop listening after the next reception
    def stop(self):
        self.listen = False

    #Legacy handle for start()
    def startListening(self):
        self.start()

    #Legacy handle for stop()
    def stopListening(self):
        self.stop()

#Creates a Server object for an event based socket system
class TimeoutServer():

    #Given a port, waits for messages on that channel. Received messages are passed to an event handler
    def __init__(self, port, eventHandlerCallback, timeoutCallback, timeout, bufferSize = 2048, errorCallback = None):
        self._port = port
        self.bufferSize = bufferSize
        self._server = socket(AF_INET, SOCK_DGRAM)
        self._server.bind(('', self._port))
        self.callback = eventHandlerCallback
        self.errorCallback = errorCallback
        self.timeoutCallback = timeoutCallback
        self.timeout = timeout
        self._server.settimeout(self.timeout)
        self.listen = False

    #Stops listening on delete in an attempt to prevent hanging
    def __del__(self):
        self._server.close()
        self.listen = False

    #Starts an event loop that passes received data back to an event handler
    def start(self):
        self.listen = True
        while self.listen:
            try:
                self._server.settimeout(self.timeout)
                message, addr = self._server.recvfrom(self.bufferSize)
                data = String2Dictionary(message)
                self.callback(data)
            except timeout:
                self.timeoutCallback()
            except:
                if not self.errorCallback == None:
                    self.errorCallback()
                continue

    #Flags the server to stop listening after the next reception
    def stop(self):
        self.listen = False

    #Legacy handle for start()
    def startListening(self):
        self.start()

    #Legacy handle for stop()
    def stopListening(self):
        self.stop()

#Creates a socket that sends joystick data to its target
class Client():

    #Requires the ip of the target and the port to use
    def __init__(self, ip, port):
        self._client = socket(AF_INET, SOCK_DGRAM)
        self._address = (ip, port)

    #Safely disposes of the socket
    def __del__(self):
        self._client.close()

    #automatically converts a dictionary to a string and sends the string
    def send(self, data):
        message = Dictionary2String(data)
        self._client.sendto(message, self._address)

#Creates a Server object for an event based socket system
class JSONServer():

    #Given a port, waits for messages on that channel. Received messages are passed to an event handler
    def __init__(self, port, eventHandlerCallback, bufferSize = 2048, errorCallback = None):
        self._port = port
        self.bufferSize = bufferSize
        self._server = socket(AF_INET, SOCK_DGRAM)
        self._server.bind(('', self._port))
        self.callback = eventHandlerCallback
        self.errorCallback = errorCallback
        self.isBlocking = True
        self.listen = False

    #Stops listening on delete in an attempt to prevent hanging
    def __del__(self):
        self._server.close()
        self.listen = False

    #Configures
    def setBlocking(self, boolean = True):
        if boolean:
            self._server.setblocking(1)
            self.isBlocking = True
        else:
            self._server.setblocking(0)
            self.isBlocking = False

    #Starts an event loop that passes received data back to an event handler
    def start(self):
        self.listen = True
        while self.listen:
            try:
                message, addr = self._server.recvfrom(2048)
                data = json.loads(message)
                self.callback(data)
            except:
                if not self.errorCallback is None:
                    self.errorCallback()
            if not self.isBlocking:
                sleep(0.01)

    #Flags the server to stop listening after the next reception
    def stop(self):
        self.listen = False

    #Legacy handle for start()
    def startListening(self):
        self.start()

    #Legacy handle for stop()
    def stopListening(self):
        self.stop()

#Creates a socket that sends joystick data to its target
class JSONClient():

    #Requires the ip of the target and the port to use
    def __init__(self, ip, port):
        self._client = socket(AF_INET, SOCK_DGRAM)
        self._address = (ip, port)

    #Safely disposes of the socket
    def __del__(self):
        self._client.close()

    #Converts data to JSON string and sends it
    def send(self, data):
        message = json.dumps(data)
        self._client.sendto(message, self._address)
