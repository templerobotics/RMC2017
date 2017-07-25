from math import ceil, sqrt
import Queue as Q
import serial
from time import sleep
import threading

useDecimalTargets = True

ONE = 1 if useDecimalTargets else 49
TWO = 2 if useDecimalTargets else 50

ADDRESSES = [128, 129]

COMMANDS = {'SET': 40,
            'GET': 41,
            'REPLY': 73,
            }

SET_COMMANDS = {'VALUE': 0,
             'ALIVE': 16,
             'SHUTDOWN': 32,
             'TIMEOUT': 64,
             }

GET_COMMANDS = {'VALUE': 0,
             'BATTERY': 16,
             'CURRENT': 32,
             'TEMPERATURE': 64,
             }

SET_TARGETS = {'M1': (77, ONE),
              'M2': (77, TWO),
              'M*': (77, 42),
              'MD': (77, 68),
              'MT': (77, 84),
              'P1': (80, ONE),
              'P2': (80, TWO),
              'P*': (80, 42),
              'Q1': (81, ONE),
              'Q2': (81, TWO),
              'Q*': (81, 42),
              'R1': (82, ONE),
              'R2': (82, TWO),
              'R*': (82, 42),
              'T1': (84, ONE),
              'T2': (84, TWO),
              'T*': (84, 42),
              }

GET_TARGETS = {'S1': (83, ONE),
              'S2': (83, TWO),
              'A1': (65, ONE),
              'A2': (65, TWO),
              'M1': (77, ONE),
              'M2': (77, TWO),
              'P1': (80, ONE),
              'P2': (80, TWO),
              }

REPLY_SIG = chr(73)

REPLY_VALUES = {0: 'VALUE',
                1: 'VALUE_NEG',
                16: 'BATTERY',
                17: 'BATTERY_NEG',
                32: 'CURRENT',
                33: 'CURRENT_NEG',
                64: 'TEMPERATURE',
                65: 'TEMPERATURE_NEG',
                }

#Creates a serial interface designed to send packets to Sabertooth 2x32 Motor Controllers
class SPSI(threading.Thread):

    def __init__(self, port):

        #Serial output
        self.com = serial.Serial(port, baudrate = 9600, timeout = 1)

        self.readQueue = Q.Queue()

        self.daemon = True

        self.start()

    #Safely releases the serial port
    def __del__(self):
        self.com.close()

    def run(self):
        while self._alive:
            response = self.com.read(6)
            if len(response) == 6 and response[0] == REPLY_SIG:
                replyType = ord(response[1])
                if replyType in REPLY_VALUES:
                    replyValue = REPLY_VALUES[replyType]
                    value = self._joinValue(response[2], response[3], replyValue.endswith('_NEG'))
                    source = response[-2:6]
                    self.readQueue.put_nowait((replyValue, value, source))
            sleep(0.5)

    #Splits integer across 2 bytes
    def _splitValue(value):
        lead = (value >> 0) & 127
        tail = (value >> 7) & 127
        return lead, tail

    #Joins a split response value into a single value
    def _joinValue(byte1, byte2, negative = False):
        return byte1 + (byte2 << 7) if not negative else -(byte1 + (byte2 << 7))

    #Expands value from [-1.0f, 1.0f] to [-2047L, 2047L]
    def _expandValue(value):
        return int(2047 * value)

    #Shrinks value from [-2047L, 2047L] to [-1.0f, 1.0f]
    def _shrinkValue(value):
        return float(value / 2047)

    #Sends the packet along with a checksum
    def _sendPacket(self, address, command, commandValue, data):
        '''Used to send a packet to a Sabertooth. Do not call directly'''

        #Computes checksums for error checking
        commandChecksum = (address + command + commandValue) & 127
        dataChecksum = sum(data) & 127

        data = [chr(i) for i in data] #Convert ints to bytes
        dataString = ''.join(data) + chr(dataChecksum) #Concatenate data bytes into bytes array
        commandString = chr(address) + chr(command) + chr(commandValue) + chr(dataChecksum) + dataString #Concatenate full command string

        #Write the command
        self.com.write(commandString)

    def set(self, address, target, value, subcommand = 'VALUE'):
        '''Sends a SET command to the Sabertooth'''
        #Fetch the appropriate command bytes
        command = COMMANDS['SET']
        commandValue = SET_COMMANDS[subcommand] + 1 if value < 0.0 else SET_COMMANDS[subcommand]

        #Get the data bytes
        targetLead, targetTail = SET_TARGETS[target]
        valueLead, valueTail = self._splitValue(self._expandValue(value))

        #Sends the bytes
        self._sendPacket(address, command, commandValue, [valueLead, valueTail, targetLead, targetTail])

    def get(self, address, target, subcommand = 'VALUE', timeout = 3.0):
        '''Sends a GET command to the Sabertooth. Returns the value, or None if timed out'''
        #Fetch the appropriate command bytes
        command = COMMANDS['GET']
        commandValue = GET_COMMANDS[subcommand]

        #Get the data bytes
        targetLead, targetTail = GET_TARGETS[target]

        #Sends the bytes
        self._sendPacket(address, command, commandValue, [targetLead, targetTail])

        replyType, value, source = self.readQueue.get(True, timeout)

        if replyType == commandValue and source == target:
            if replyType == 'VALUE':
                return self._shrinkValue(value)
            else:
                return value
        else:
            return None

class Motor:

    def __init__(self, address, motorNumber, invertInput = False, throttle = 1.0, ramping = 0.25):
        global _spsi

        self.com = _spsi

        if not (address == 128 or address == 129):
            raise ValueError('Address must be 128 or 129')

        if not (motorNumber == 1 or motorNumber == 2):
            raise ValueError('Motor Number must be 1 or 2')

        self._address = address
        self._motor = 'M' + str(motorNumber)
        self._power = 'P' + str(motorNumber)
        self._q = 'Q' + str(motorNumber)
        self._ramping = 'R' + str(motorNumber)

        self.invert = invertInput

        self.setSpeed(0.0)
        self.setThrottle(throttle)
        self.setRamping(ramping)

    def __del__(self):
        '''Stops the motor'''
        self.setSpeed(0.0)

    def _convertValue(self, value):
        '''Converts value from range [0.0, 1.0] to [-1.0, 1.0] for the packet values'''
        return float(2.0 * value - 1.0)

    def stop(self):
        '''Shortcut for setSpeed(0.0)'''
        self.setSpeed(0.0)

    def setSpeed(self, speed = 0.0):
        '''Sets the speed of the motor. Input should be in range [-1.0, 1.0]'''

        #Forces speed into valid range
        if speed < -1.0:
            speed = -1.0
        elif speed > 1.0:
            speed = 1.0

        #Inverts speed
        if self.invert: speed = -speed

        self.speed = speed

        #Sends the speed command
        self.com.set(self.address, self._motor, speed)

    def setThrottle(self, throttle = 1.0):
        '''Sets the max power of the motor. Input should be in range [0.0, 1.0]'''

        if throttle < 0.0:
            throttle = 0.0
        elif throttle > 1.0:
            throttle = 1.0

        self.throttle = throttle

        throttle = self._convertValue(throttle)

        self.com.set(self._address, self._power, throttle)

    def setRamping(self, ramping = 0.25):
        '''Sets the speed ramping of the motor. Input should be in range [0.0, 1.0]'''

        if ramping < 0.0:
            ramping = 0.0
        elif ramping > 1.0:
            ramping = 1.0

        self.ramping = ramping

        ramping = self._convertValue(ramping)

        self.com.set(self._address, self._ramping, ramping)

    def getBatteryVoltage(self, timeout = 1.5):
        '''Returns the approximate input voltage to the Sabertooth'''
        value = self.com.get(self._address, self._motor, 'BATTERY', timeout)
        if value is not None:
            return value / 10
        else:
            return None

    def getCurrentUsage(self, timeout = 1.5):
        '''Returns the approximate current draw of this motor. Very approximate'''
        value = self.com.get(self._address, self._motor, 'CURRENT', timeout)
        if value is not None:
            return value / 10
        else:
            return None

    def getTemperature(self, timeout = 1.5):
        '''Returns the temperature of this motor's transistors in Celsius'''
        value = self.com.get(self._address, self._motor, 'TEMPERATURE', timeout)
        return value

class LinearActuator(Motor):

    def __init__(self, address, motorNumber, invertCommands = False, throttle = 1.0, ramping = 0.25):
        super(LinearActuator, self).__init__()

    def extend(self):
        '''Shortcut for setSpeed(1.0)'''
        self.setSpeed(1.0)

    def retract(self):
        '''Shortcut for setSpeed(-1.0)'''
        self.setSpeed(-1.0)

class Drivetrain:

    def __init__(self, address, throttle = 1.0, ramping = 0.25, invertX = False, invertY = False, swapInputs = False):
        global _spsi

        self.com = _spsi

        if not (address == 128 or address == 129):
            raise ValueError('Address must be 128 or 129')

        self._address = address
        self._motor_drive = 'MD'
        self._motor_turn = 'MT'
        self._power = 'P*'
        self._q = 'Q*'
        self._ramping = 'R*'

        self.invertX = invertX
        self.invertY = invertY
        self.swapInputs = swapInputs

        self._setSpeed(0.0)
        self._setTurn(0.0)
        self.setThrottle(throttle)
        self.setRamping(ramping)

    def __del__(self):
        '''Stops the motor'''
        self.setSpeed(0.0)

    def _convertValue(self, value):
        '''Converts value from range [0.0, 1.0] to [-1.0, 1.0] for the packet values'''
        return float(2.0 * value - 1.0)

    def stop(self):
        '''Shortcut for drive(0.0, 0.0)'''
        self.drive(0.0, 0.0)

    def _setSpeed(self, speed = 0.0):
        '''Sets the drive speed of the drivetrain. Input should be in range [-1.0, 1.0]'''

        #Forces speed into valid range
        if speed < -1.0:
            speed = -1.0
        elif speed > 1.0:
            speed = 1.0

        #Inverts speed
        if self.invert: speed = -speed

        self.speed = speed

        #Sends the speed command
        self.com.set(self._address, self._motor_drive, speed)

    def _setTurn(self, speed = 0.0):
        '''Sets the turn speed of the drivetrain. Input should be in range [-1.0, 1.0]'''

        #Forces speed into valid range
        if speed < -1.0:
            speed = -1.0
        elif speed > 1.0:
            speed = 1.0

        #Inverts speed
        if self.invert: speed = -speed

        self.turn = speed

        #Sends the speed command
        self.com.set(self._address, self._motor_turn, speed)

    def drive(self, x = 0.0, y = 0.0):
        '''Drives the drivetrain via a tank-style system'''

        if self.invertX: x = -x
        if self.invertY: y = -y

        if not self.swapInputs:
            self._setSpeed(y)
            self._setTurn(x)
        else:
            self._setSpeed(x)
            self._setTurn(y)

    def setThrottle(self, throttle = 1.0):
        '''Sets the max power of the drivetrain. Input should be in range [0.0, 1.0]'''

        if throttle < 0.0:
            throttle = 0.0
        elif throttle > 1.0:
            throttle = 1.0

        self.throttle = throttle

        throttle = self._convertValue(throttle)

        self.com.set(self._address, self._power, throttle)

    def setRamping(self, ramping = 0.25):
        '''Sets the speed ramping of the drivetrain. Input should be in range [0.0, 1.0]'''

        if ramping < 0.0:
            ramping = 0.0
        elif ramping > 1.0:
            ramping = 1.0

        self.ramping = ramping

        ramping = self._convertValue(ramping)

        self.com.set(self._address, self._ramping, ramping)

    def getBatteryVoltage(self, timeout = 1.5):
        '''Returns the approximate input voltage to the Sabertooth'''
        value = self.com.get(self._address, 'M1', 'BATTERY', timeout)
        if value is not None:
            return value / 10
        else:
            return None

    def getCurrentUsage(self, motorNumber = 1, timeout = 1.5):
        '''Returns the approximate current draw of this motor. Very approximate'''
        motor = ''
        if motorNumber = 1:
            motor = 'M1'
        elif motorNumber = 2:
            motor = 'M2'
        else:
            raise ValueError('Motor number must be 1 or 2')
        value = self.com.get(self._address, motor, 'CURRENT', timeout)
        if value is not None:
            return value / 10
        else:
            return None

    def getTemperature(self, motorNumber = 1, timeout = 1.5):
        '''Returns the temperature of this motor's transistors in Celsius'''
        motor = ''
        if motorNumber = 1:
            motor = 'M1'
        elif motorNumber = 2:
            motor = 'M2'
        else:
            raise ValueError('Motor number must be 1 or 2')
        value = self.com.get(self._address, motor, 'TEMPERATURE', timeout)
        return value

def init():
    global _spsi
    _spsi = SPSI('/dev/ttyS0')

if __name__ == 'sabertooth_serial_alt32':
    init()
