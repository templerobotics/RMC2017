from ast import literal_eval
from socket import *
import threading

class _RDTPServer(threading.Thread):

    def __init__(self, port):
        
