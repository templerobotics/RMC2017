from ast import literal_eval
import errno
from socket import *
from Queue import Queue
import threading
import time

BDTP_SIG = {'EST': chr(124), 'ACK': chr(125), 'REJ': chr(126), 'END': chr(127)}

class BDTP(threading.Thread):

    def __init__(self, ip, port = 27000, bufferSize = 512, establishTimeout = 30):
        threading.Thread.__init__(self)

        self.address = (ip, port)

        self.messageQueue = Queue()
        self.messageAvailableEvent = thread.Event()
        self.connectionEvent = thread.Event()

        self._server = socket(AF_INET, SOCK_DGRAM)
        self._server.bind(('', self.address[1]))

        self._client = socket(AF_INET, SOCK_DGRAM)

        self._connected = False
        self.remoteAddress = ('', -1)
        self.remoteBufferSize = -1

        self.start()

    def __del__(self):
        self._server.close()
        self._client.close()
        self._alive = False

    def run(self):
        self._establishConnection()

        while self._alive:
            try:
                data, addr = self._server.recvfrom(self.bufferSize)
                if self._acceptAddress(addr):
                    if data = BDTP_SIG['END']:
                        self._closeConnection()
                    else:
                        self._decode(data)
                else:
                    self._client.sendto(BDTP_SIG['REJ'], addr)
            except timeout:
                continue
            except error as e:
                if e[0] == errno.EMSGSIZE:
                    continue
                else:
                    raise e
            except Exception as err:
                raise err

    def _establishConnection(self):

        receivedREQ = False
        receivedACK = False

        myReq = BDTP_SIG['REQ'] + str(self.bufferSize) + str(time.time() % 10)
        myACK = BDTP_SIG['ACK'] + str(self.bufferSize) + str(time.time() % 10)

        self._server.settimeout(1.0)
        self._client.sendto(myREQ, self.address)

        while not (receivedACK and receivedREQ):
            try:
                message, addr = self._server.recvfrom(self.bufferSize)
            except timeout:
                if not receivedACK:
                    self._client.sendto(myREQ, self.address)
            if not receivedREQ and message[0] == BDTP_SIG['REQ'] and addr[0] == self.address[0]:
                try:
                    self.remoteBufferSize = int(message[1:-1])
                    self.remoteAddress = addr
                    receivedREQ = True
                    self._client.sendto(BDTP_SIG['ACK'] + message[1:], self.address)
                except ValueError:
                    continue
            elif not receivedACK and message == myACK:
                receivedACK = True

        self._connected = True
        self._server.settimeout(None)
        self.connectionEvent.set()

    def _acceptAddress(self, addr):
        return self.remoteAddress[0] == addr[0] and self.remoteAddress[1] == addr[1]

    def _checksum(self, string):
        total = 0
        for letter in string:
            total += ord(letter)
        return chr(total)

    def _encode(self, message):
        return message + self._checksum(message)

    def _decode(self, message):
        if len(message) > 1:
            data, parity = message[:-1], message[-1]
            if parity == self._checksum(data):
                self.messageQueue.put(literal_eval(data))
                self.messageAvailableEvent.set()

    def isSendable(self, data):
        string = repr(data).replace(' ', '')
        return len(string) < self.remoteBufferSize

    def send(self, data):
        string = repr(data).replace(' ', '')
        if len(string) < self.remoteBufferSize:
            self._client.sendto(self._encode(string), self.address)
        else:
            raise ValueError('String exceeds remote buffer size')

    def receive(self):
        try:
            return self.messageQueue.get_nowait()
        except:
            return None
