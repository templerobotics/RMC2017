#!/usr/bin/env python

#Sam Wilson 1/27/17
#Takeover version

import pigpio
import wavePWM
import subprocess
from time import sleep

#Initialize a global referencing the pigpio daemon
try:
    PIGPIO_DAEMON = pigpio.pi()
except:
    #Starts the pigpio daemon if pigpio fails to bind to the pi
    subprocess.call("sudo pigpiod", shell=True)

#Legacy handle for SabretoothLinearActuator, which used to be LinearActuator
def LinearActuator(pin):
    return SabretoothLinearActuator(pin)

#A generic PWM generator object. Useful for testing
class PWMWave:

    #The pin number should be the BCM/GPIO pin number
    def __init__(self, pin):
        self._pin = pin

        self._wave = wavePWM.PWM(PIGPIO_DAEMON)

        self._wave.set_cycle_time(1000)
        self._wave.set_pulse_length_in_micros(self._pin, 500)

        self._wave.update()

        self._cycleTime = 1000
        self._frequency = 1000

        self._pulseWidth = 500
        self._dutyCycle = 0.5

    def __del__(self):
        self._wave.cancel()

    #Sets the frequency in Hertz
    def setFrequency(self, frequency):
        self._wave.set_frequency(frequency)
        self._wave.update()

        self._frequency = frequency
        self._cycleTime = 1000000.0 / self._frequency

    #Sets the cycle time in microseconds
    def setCycleTime(self, time):
        self._wave.set_cycle_time(time)
        self._wave.update()

        self._cycleTime = time
        self._frequency = 1000000.0 / self._cycleTime

    #Sets the duty cycle as a value from 0.0 to 1.0
    def setDutyCycle(self, fraction):
        self._wave.set_pulse_length_in_fraction(self._pin, fraction)
        self._wave.update()

        self._dutyCycle = fraction
        self._pulseWidth = self._cycleTime * fraction

    #Sets the length of the pulse in microseconds
    def setPulseTime(self, time):
        self._wave.set_pulse_length_in_micros(self._pin, time)
        self._wave.update()

        self._pulseWidth = time
        self._dutyCycle = self._cycleTime / self._pulseWidth

    #Returns wave frequency in Hertz
    def getFrequency(self):
        return self._frequency

    #Returns cycle time in microseconds
    def getCycleTime(self):
        return self._cycleTime

    #Returns duty cycle as a value from 0.0 to 1.0
    def getDutyCycle(self):
        return self._dutyCycle

    #Returns pulse time in microseconds
    def getPulseTime(self):
        return self._pulseWidth

#For use with DC motors via Sabretooth controllers. Sabretooth should be in RC microcontroller mode
class SabretoothDCMotor:

    #The pin number should be the BCM/GPIO pin number
    def __init__(self, pin):
        self._pin = pin

        self._wave = wavePWM.PWM(PIGPIO_DAEMON)
        self._wave.set_cycle_time(3000)

        self._currentPulseWidth = 1500

        self.setSpeed(0.0)
        self._isSuspended = False

    #Makes sure to stop the motor and cancel the PWM before garbage collection
    def __del__(self):
        self.suspend()

    #Stops the calling object from generating PWM signals. Stopping safely means setting the speed to 0 before suspending
    def suspend(self, safely = True):
        if not self._isSuspended:
            if safely:
                self.setSpeed(0.0)
            self._wave.cancel()
            self._isSuspended = True

    #Reinitializes the motor object after having been suspended
    def resume(self, resumeSpeed = False):
        if self._isSuspended:
            self._wave = wavePWM.PWM(PIGPIO_DAEMON)
            self._wave.set_cycle_time(3000)
            self._isSuspended = False
            if resumeSpeed:
                self.setSpeed(self.getSpeed)
            else:
                self.setSpeed(0.0)

    #Takes a value from -1.0 to 1.0 to set the speed of the motor
    def setSpeed(self, speed):
        if self._isSuspended:
            self.resume()

        if speed < -1.0:
            speed = -1.0
        elif speed > 1.0:
            speed = 1.0

        speed = (speed + 3.0) * 500.0

        self._wave.set_pulse_length_in_micros(self._pin, speed)
        self._wave.update()

        self._currentPulseWidth = speed

        return speed

    #Returns the speed of the motor from -1.0 to 1.0
    def getSpeed(self):
        return (self._currentPulseWidth / 500.0) - 3.0

    #Runs a gradient from the current speed to a target speed. Number of steps and time of gradient are customizable
    def autoGradient(self, targetSpeed = 0.0, steps = 20, time = 1.0):
        if self._isSuspended:
            self.resume()

        if targetSpeed < -1.0:
            targetSpeed = -1.0
        elif targetSpeed > 1.0:
            targetSpeed = 1.0

        if targetSpeed == self.getSpeed():
            return (targetSpeed + 3.0) * 500.0

        targetSpeed = (targetSpeed + 3.0) * 500.0

        gradient = range(int(self._currentPulseWidth), int(targetSpeed), int((targetSpeed - self._currentPulseWidth) / steps))

        for speed in gradient:
            self._wave.set_pulse_length_in_micros(self._pin, speed)
            self._wave.update()
            sleep(float(time / steps))

        self._wave.set_pulse_length_in_micros(self._pin, targetSpeed)
        self._wave.update()

        self._currentPulseWidth = targetSpeed

        return targetSpeed

#For use with linear actuators via Sabretooth controllers. Sabretooth should be in RC microcontroller mode
class SabretoothLinearActuator:

    #The pin number should be the BCM/GPIO pin number
    def __init__(self, pin, invertDirections = False):
        self._pin = pin

        self._wave = wavePWM.PWM(PIGPIO_DAEMON)
        self._wave.set_cycle_time(3000)

        self._currentPulseWidth = 1500

        if invertDirections:
            self._extendValue = 2000
            self._retractValue = 1000
        else:
            self._extendValue = 1000
            self._retractValue = 2000

        self.idle()
        self._isSuspended = False

    #Makes sure to stop the motor and cancel the PWM before garbage collection
    def __del__(self):
        self.suspend()

    #Stops the calling object from generating PWM signals. Stopping safely means setting the speed to 0 before suspending
    def suspend(self, safely = True):
        if not self._isSuspended:
            if safely:
                self.idle()
            self._wave.cancel()
            self._isSuspended = True

    #Reinitializes the actuator object after having been suspended
    def resume(self, resumeState = False):
        if self._isSuspended:
            self._wave = wavePWM.PWM(PIGPIO_DAEMON)
            self._wave.set_cycle_time(3000)
            self._isSuspended = False
            if resumeState:
                if self._isExtended:
                    self.extend()
                elif not self._isExtended:
                    self.retract()
                else:
                    self.idle()

    #Sets the actuator to the extended value at maximum available speed
    def extend(self):
        if self._isSuspended:
            self.resume()

        self._wave.set_pulse_length_in_micros(self._pin, self._extendValue)
        self._wave.update()

        self._isExtended = True
        self._state = 'extended'

    #Sets the actuator to the retract value at maximum available speed
    def retract(self):
        if self._isSuspended:
            self.resume()

        self._wave.set_pulse_length_in_micros(self._pin, self._retractValue)
        self._wave.update()

        self._isExtended = False
        self._state = 'retracted'

    #Leaves the actuator in whichever position it was in, but deactivates its resistance force
    def idle(self):
        if self._isSuspended:
            self.resume()

        self._wave.set_pulse_length_in_micros(self._pin, 1500)
        self._wave.update()

        self._state = 'idle'

    #Inverts the extend and retract values of this actuator
    def invert(self):
        if self._extendValue == 1000:
            self._extendValue = 2000
            self._retractValue = 1000
        else:
            self._extendValue = 1000
            self._retractValue = 2000

    #Returns a boolean value of whether or not the actuator is extended. If called right after __init__ and before any calls to extend or retract, the value of isExtended will be None
    def isExtended(self):
        return self._isExtended

    #Returns one of three strings describing the current state: 'extended', 'retracted', or 'idle'
    def getState(self):
        return self._state

#Creates a drivetrain object allowing joystick values to automatically be formatted for the drive motors.
#For single joystick, use arcadeDrive; for dual joystick use tankDrive.
class RobotDrivetrain:

    #Any two motor objects can be used, so long as they have updateSpeed() and suspend() methods
    def __init__(self, rightMotor, leftMotor):
        self._rightMotor = rightMotor
        self._leftMotor = leftMotor

    #Stops PWM generation to prevent motor twitching. Use when the robot needs to sit still
    def suspend(self):
        self._rightMotor.suspend()
        self._leftMotor.suspend()

    #Use for a single joystick setup controlling a tank-style drivetrain
    #See http://home.kendra.com/mauser/Joystick.html for the math behind this
    def arcadeDrive(self, rawStickX, rawStickY, invertX = False, invertY = True, swapVW = False, swapRL = True):
        if invertX:
            rawStickX *= -1.0
        if invertY:
            rawStickY *= -1.0

        if not swapVW:
            v = (1.0 - abs(rawStickX)) * rawStickY + rawStickY
            w = (1.0 - abs(rawStickY)) * rawStickX + rawStickX
        else:
            w = (1.0 - abs(rawStickX)) * rawStickY + rawStickY
            v = (1.0 - abs(rawStickY)) * rawStickX + rawStickX

        if not swapRL:
            rSpeed = (v + w) / 2.0
            lSpeed = (v - w) / 2.0
        else:
            lSpeed = (v + w) / 2.0
            rSpeed = (v - w) / 2.0

        self._rightMotor.setSpeed(rSpeed)
        self._leftMotor.setSpeed(lSpeed)

        return (rSpeed, lSpeed)

    #Use for a dual joystick setup controlling a tank-style drivetrain
    def tankDrive(self, rawStickLeft, rawStickRight, invertLeft = False, invertRight = False):
        if invertLeft:
            rawStickLeft *= -1.0
        if invertRight:
            rawStickRight *= -1.0

        self._rightMotor.setSpeed(rawStickRight)
        self._leftMotor.setSpeed(rawStickLeft)

        return (rawStickRight, rawStickLeft)

#Allows a number of actuators to be controlled as a group
class ActuatorGroup:

    #The actuatorList can be a standard list, tuple, or dictionary of LinearActuator objects
    def __init__(self, actuatorList, invertDirections = False):
        self._actuators = actuatorList
        self.idleAll()

    #Makes sure to stop the actuators and cancel the PWM before garbage collection
    def __del__(self):
        for actuator in self._actuators:
            del actuator

    #Stops a specific actuator from generating PWM signals. Stopping safely means setting the speed to 0 before suspending
    def suspend(self, index = 'all', safely = True):
        if index == 'all':
            self.suspendAll(safely)
        else:
            actuator = self._actuators[index]
            actuator.suspend(safely)

    #Stops all actuators from generating PWM signals. Stopping safely means setting the speed to 0 before suspending
    def suspendAll(self, safely = True):
        for actuator in self._actuators:
            actuator.suspend(safely)

    #Reinitializes the actuator object after having been suspended
    def resume(self, index = 'all', resumeState = False):
        if index == 'all':
            self.resumeAll(resumeState)
        else:
            actuator = self._actuators[index]
            actuator.resume(resumeState)

    #Reinitializes all actuator objects after suspension
    def resumeAll(self, resumeState = False):
        for actuator in self._actuators:
            actuator.resume(resumeState)

    #Sets a specific actuator to the extended value
    def extend(self, index = 'all'):
        if index == 'all':
            self.extendAll()
        else:
            actuator = self._actuators[index]
            actuator.extend()

    #Sets all actuators to the extended value
    def extendAll(self):
        for actuator in self._actuators:
            actuator.extend()

    #Sets a specific actuator to the retracted value
    def retract(self, index = 'all'):
        if index == 'all':
            self.retractAll()
        else:
            actuator = self._actuators[index]
            actuator.retract()

    #Sets all actuators to the retract value
    def retractAll(self):
        for actuator in self._actuators:
            actuator.retract()

    #Sets a specific actuator to idle
    def idle(self, index = 'all'):
        if index == 'all':
            self.idleAll()
        else:
            actuator = self._actuators[index]
            actuator.idle()

    #Sets all actuators to idle
    def idleAll(self):
        for actuator in self._actuators:
            actuator.idle()

    #Inverts the directions of all actuators
    def invert(self, index = 'all'):
        if index == 'all':
            self.invertAll()
        else:
            actuator = self._actuators[index]
            actuator.invert()

    #Inverts the directions of all actuators
    def invertAll(self):
        for actuator in self._actuators:
            actuator.invert()

    #Returns a boolean value of whether or not the actuator is extended. If called right after __init__ and before any calls to extend or retract, the value of isExtended will be None
    def isExtended(self, index):
        return self._actuators[index].isExtended()

    #Returns one of three strings describing the current state: 'extended', 'retracted', or 'idle'
    def getState(self, index):
        return self._actuators[index].getState()
