#! python2

import shutil
import requests
from time import sleep

#If anyone figures out other commands, feel free to add them to this list
commandList = {'up': 0,
               'stop': 1,
               'down': 2,
               '': 3,
               'left': 4,
               '': 5,
               'right': 6 }

#Sam Wilson 10/20/16
class EasyNCamera:
    #Initializing with no arguments will use the camera's default stuff.
    #Unless the camera is changed somehow, you shouldn't need to use parameters.
    def __init__(self, IPAddress = '192.168.0.104', port = 81):
        self.IP = IPAddress
        self.PORT = str(port)

    #Go ahead and call this if you want, but I made you functions below.
    #But if you really enjoy things difficult, be my guest, lol
    def _sendCommand(self, command):
        comNum = commandList[command]
        url = 'http://{IPADDRESS}:{PORT}/decoder_control.cgi?command={COMMAND}&user=admin&pwd='.format(IPADDRESS = self.IP, PORT = self.PORT, COMMAND = comNum)
        response = requests.get(url)

    #Stops the camera's motion
    def stop(self):
        self._sendCommand('stop')

    #Duration is in seconds and optional. Any increments smaller than 0.1
    #can be affected by your system's accuracy, so don't rely on milliseconds
    def turnUp(self, duration = 0):
        self._sendCommand('up')
        if duration > 0:
            time.sleep(duration)
            self._stop()
            
    #Same comment as turnUp
    def turnDown(self, duration = 0):
        self._sendCommand('down')
        if duration > 0:
            time.sleep(duration)
            self._stop()
            
    #Same comment as turnUp
    def turnRight(self, duratio = 0):
        self._sendCommand('right')
        if duration > 0:
            time.sleep(duration)
            self._stop()
            
    #Same comment as turnUp
    def turnLeft(self, duration = 0):
        self._sendCommand('left')
        if duration > 0:
            time.sleep(duration)
            self._stop()
            

    #If you don't specify a path, the image will be stored as img.png in the directory of this script
    def getImage(self, path = 'img.png'):
        url = 'http://{IPADDRESS}:{PORT}/snapshot.cgi?user=admin&pwd='.format(IPADDRESS = self.IP, PORT = self.PORT)
        response = requests.get(url, stream = True)
        if response.status_code == 200:
            with open(path, 'wb') as f:
                shutil.copyfileobj(response.raw, f)
        return response.status_code
        
