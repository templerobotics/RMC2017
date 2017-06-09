from ast import literal_eval
from Queue import *
from socket import *
import threading

#Creates a Server object for a multithreaded socket system. Messages are put into a FiFo queue
class Server(threading.Thread):

    #Takes in a port to listen on. Received messages will be put in a FiFo queue
    def __init__(self, port, bufferSize = 2048, queueMaxSize = -1, bindAddress = ''):
        threading.Thread.__init__(self)

        self.messageQueue = Queue(maxsize = queueMaxSize)
        self.messageAvailableEvent = threading.Event()
        self.messageOverflowEvent = threading.Event()

        self._bindAddress = bindAddress
        self._port = port
        self.bufferSize = bufferSize
        self._socket = socket(AF_INET, SOCK_DGRAM)
        self._socket.bind((self._bindAddress, self._port))

        self.listen = False
        self._alive = True

    #Attempts to safely close the socket
    def __del__(self):
        self._socket.close()

    #Main thread loop. Started by calling the start() method of the Server object
    def run(self):
        self.listen = True
        while self._alive:
            if self.listen:
                message, addr = self._socket.recvfrom(self.bufferSize)
                try:
                    #Evaluates the message as python code and puts it in the messageQueue
                    self.messageQueue.put_nowait(literal_eval(message))
                    self.messageOverflowEvent.clear()
                except Full:
                    self.messageOverflowEvent.set()
            if self.messageQueue.empty():
                self.messageAvailableEvent.clear()
            else:
                self.messageAvailableEvent.set()
        self._socket.close()

    #Flags the Server to shutdown on the next pass. Irreversible.
    def kill(self):
        self._alive = False

#Creates a Server object for a multithreaded socket system. Messages are put into a LiFo queue
class StackingServer(Server):

    #Takes in a port to listen on. Received messages will be put in a LiFo queue
    def __init__(self, port, bufferSize = 2048, queueMaxSize = -1, bindAddress = ''):
        Server.__init__(self, port, bufferSize, queueMaxSize, bindAddress)
        self.messageQueue = LiFoQueue(maxsize = queueMaxSize)

#Creates a Server object for a multithreaded socket system. Messages are put into a priority queue
class PriorityServer(Server):

    #Takes in a port to listen on. Received messages will be put in a priority queue
    def __init__(self, port, bufferSize = 2048, queueMaxSize = -1, bindAddress = ''):
        Server.__init__(self, port, bufferSize, queueMaxSize, bindAddress)
        self.messageQueue = PriorityQueue(maxsize = queueMaxSize)

#Creates a Client object for a multithreaded socket system. Messages are put into a FiFo queue
class Client(threading.Thread):

    #Takes in an IP address and port to send data to
    def __init__(self, ip, port):
        threading.Thread.__init__(self)

        self._socket = socket(AF_INET, SOCK_DGRAM)
        self._address = (ip, port)

        self.messageQueue = Queue()
        self.busyEvent = threading.Event()

        self._alive = True

    #Attempts to safely close the socket
    def __del__(self):
        self._socket.close()

    #Main thread loop. Started by calling the start() method of the Client object
    def run(self):
        while self._alive:
            try:
                data = self.messageQueue.get(timeout = 1)
                self.busyEvent.set()
                message = repr(data).replace(' ', '')
                self._socket.sendto(message, self._address)
                self.messageQueue.task_done()
            except Empty:
                self.busyEvent.clear()
        self._socket.close()

    #Function wrapper to enqueue data to be sent
    def queueToSend(data):
        self.messageQueue.put_nowait(data)

    #Flags the Server to shutdown on the next pass. Irreversible.
    def kill(self):
        self._alive = False

#Creates a Client object for a multithreaded socket system. Messages are put into a LiFo queue
class StackingClient(Client):

    #Takes in an IP address and port to send data to
    def __init__(self, ip, port):
        Client.__init__(self, ip, port)
        self.messageQueue = LiFoQueue()

#Creates a Client object for a multithreaded socket system. Messages are put into a priority queue
class PriorityClient(Client):

    #Takes in an IP address and port to send data to
    def __init__(self, ip, port):
        Client.__init__(self, ip, port)
        self.messageQueue = PriorityQueue()
