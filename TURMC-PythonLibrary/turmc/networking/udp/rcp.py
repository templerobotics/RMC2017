from socket import *
from threading import *
from Queue import *
from time import time

RCP_BUFF = 1024
RCP_PORT = 21000
RCP_TIMEOUT = 3.0
RCP_SIG = {'NUL': chr(0),
           'NEW': chr(1),
           'BYE': chr(2),
           'PIN': chr(3),
           'PON': chr(4),
           'ALV': chr(5),
           'RRQ': chr(6),
           'WRQ': chr(7),
           'IRQ': chr(8),
           'ACK': chr(12),
           'BOF': chr(13),
           'EOF': chr(14),
           'BOS': chr(15),
           'EOS': chr(16)}


def checksum(string):
    total = 0
    for character in string:
        total += ord(character)
    return chr(total % 128)

def validate(string):
    return checksum(string[:-1]) == string[-1]

class RCP(Thread):

    def __init__(self, hostname):
        Thread.__init__(self)

        self.hostname = hostname

        self._opentime = time()
        self._bytessent = 0
        self._bytesrecv = 0

        self._server = socket(AF_INET, SOCK_DGRAM)
        self._client = socket(AF_INET, SOCK_DGRAM)
        self._server.bind(('', RCP_PORT))

        self._hostnames = {}
        self._leasetimes = {}

        self.pong = Event()

        self.start()

    def run(self):
        self._alive = True
        self._send(RCP_SIG['NEW'] + self.hostname)

        while self._alive:
            try:
                message, sender = self._recv()
            except timeout:
                self._send(RCP_SIG['ALV'])
            except KeyboardInterrupt as ki:
                self._alive = False
                raise ki

            sig, data = message[0], message[1:]

            if sig == RCP_SIG['ALV']:
                #Process alive
            elif sig == RCP_SIG['PIN']:
                self._send(RCP_SIG['PON'] + data, sender)
            elif sig == RCP_SIG['PON']:
                self.pong.set()

    def _send(self, message, address):
        message += checksum(message)
        self._bytessent += 28 + len(message)
        self._client.sendto(message, address)

    def _recv(self):
        message, address = self._server.recvfrom(RCP_BUFF)
        self._bytesrecv += 28 + len(message)
        if validate(message):
            return message[:-1], address
        else:
            return RCP_SIG['NUL'], address

    def ping(self, hostname, timeout = 10):
        if hostname in self._hostnames:
            self.pong.clear()
            startTime = time()
            self._send(RCP_SIG['PIN'], self._hostnames[hostname])
            if self.pong.wait(timeout):
                totalTime = time() - startTime
                self.pong.clear()
                return True, totalTime
            else:
                return False, -1
        else:
            raise KeyError('Hostname not known')

class _Connection:

    def __init__(self, hostname, address):
        self.hostname = hostname
        self.address = address
        self.time = time()
