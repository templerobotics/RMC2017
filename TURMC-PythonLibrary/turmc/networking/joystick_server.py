from socket import *
from ..tools.textutils import String2Dictionary

#Creates a Server object for an event based socket system
class Server():

    #Given a port, waits for messages on that channel. Received messages are passed to an event handler
    def __init__(self, port, eventHandlerCallback):
        self._server = socket(AF_INET, SOCK_DGRAM)
        self._server.bind(('', port))
        self.callback = eventHandlerCallback
        self.listen = False

    #Stops listening on delete in an attempt to prevent hanging
    def __del__(self):
        self.listen = False

    #Starts an event loop that passes received data back to an event handler
    def start(self):
        self.listen = True
        while self.listen:
            message, addr = self._server.recvfrom(2048)
            data = String2Dictionary(message)
            self.callback(data)

    #Flags the server to stop listening after the next reception
    def stop(self):
        self.listen = False

    #Legacy handle for start()
    def startListening(self):
        self.start()

    #Legacy handle for stop()
    def stopListening(self):
        self.stop()
