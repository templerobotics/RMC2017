from ast import literal_eval
import errno
from socket import *
from Queue import Queue
import threading
import time

BDTP_SIG = {'REQ': chr(124), 'ACK': chr(125), 'REJ': chr(126), 'END': chr(127)}

class BDTP(threading.Thread):

    def __init__(self, ip, port = 27000, bufferSize = 1024, establishTimeout = 30):
        threading.Thread.__init__(self)

        self.address = (ip, port)
        self.bufferSize = bufferSize + 1

        self.messageQueue = Queue()
        self.messageAvailableEvent = threading.Event()
        self.connectionEvent = threading.Event()

        self._alive = True

        self._server = socket(AF_INET, SOCK_DGRAM)
        self._server.bind(('', self.address[1]))

        self._client = socket(AF_INET, SOCK_DGRAM)

        self._connected = False
        self.remoteAddress = (self.address[0], -1)
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
                    if data == BDTP_SIG['END']:
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
        print('Establishing connection')

        self.rec_req = False
        self.rec_ack = False

        self.my_ack = BDTP_SIG['ACK'] + str(self.bufferSize)

        self._server.settimeout(1.0)
        self._client.sendto(BDTP_SIG['REQ'], self.address)

        while not self.rec_ack:
            try:
                message, addr = self._server.recvfrom(self.bufferSize)
            except timeout:
                print('Timed out')
                if not self.rec_req:
                    print('Sending REQ')
                    self._client.sendto(BDTP_SIG['REQ'], self.address)
                elif self.rec_req and not self.rec_ack:
                    print('Sending REC ACK')
                    self._client.sendto(self.my_ack, self.address)
                continue
            except KeyboardInterrupt:
                raise KeyboardInterrupt

            print('Received something: {}'.format(message))

            if addr[0] == self.address[0]:
                if message == BDTP_SIG['REQ']:
                    self.rec_req = True
                    self.remoteAddress = addr
                    print('Sending REC ACK')
                    self._client.sendto(self.my_ack, self.address)
                elif message[0] == BDTP_SIG['ACK']:
                    self.remoteBufferSize = message[1:]
                    self.remoteAddress = addr
                    self.rec_ack = True
                    self._client.sendto(self.my_ack, self.address)

            if not self.rec_req:
                print('Sending REQ')
                self._client.sendto(BDTP_SIG['REQ'], self.address)
            elif self.rec_req and not self.rec_ack:
                print('Sending REC ACK')
                self._client.sendto(self.my_ack, self.address)

        self._connected = True
        self._server.settimeout(None)
        self.connectionEvent.set()

    def _acceptAddress(self, addr):
        return self.remoteAddress[0] == addr[0] and self.remoteAddress[1] == addr[1]

    def _checksum(self, string):
        total = 0
        for letter in string:
            total += ord(letter)
        return chr(total % 256)

    def _encode(self, message):
        return message + self._checksum(message)

    def _decode(self, message):
        if len(message) > 1:
            data, parity = message[:-1], message[-1]
            if parity == self._checksum(data):
                self.messageQueue.put(literal_eval(data))
                self.messageAvailableEvent.set()

    def isSendable(self, data):
        string = repr(data)
        return len(string) < self.remoteBufferSize

    def send(self, data):
        string = repr(data)
        if len(string) < self.remoteBufferSize:
            self._client.sendto(self._encode(string), self.address)
        else:
            raise ValueError('String exceeds remote buffer size')

    def receive(self):
        try:
            return self.messageQueue.get_nowait()
        except:
            return None

    def close(self):
        self._alive = False
