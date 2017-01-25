#!/usr/bin/env python

#Sam Wilson 1/24/17
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
class LinearActuator:

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

    #Returns a boolean value of whether or not the actuator is extended. If called right after __init__ and before any calls to extend or retract, the value of isExtended will be None
    def isExtended(self):
        return self._isExtended

    #Returns one of three strings describing the current state: 'extended', 'retracted', or 'idle'
    def getState(self):
        return self._state

#Creates a drivetrain object allowing joystick values to automatically be formatted for the drive motors. 
#For single joystick, use arcadeDrive; for dual joystick use tankDrive.
class RobotDrive:

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
