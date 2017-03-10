#!/usr/bin/env python





#DO NOT USE THIS SCRIPT, FUNDAMENTAL PROBLEM. See Sam Wilson for explanation.





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
        
        self._wave.set_pulse_length_in_micros(self._pin, 1500)
        self._wave.update()
        self._isSuspended = False

    #Stops the calling object from generating PWM signals. Stopping safely means setting the speed to 0 before suspending
    def suspend(self, safely = True):
        if not self._isSuspended:
            if safely:
                self.setSpeed(0.0)
            self._wave.cancel()
            self._isSuspended = True

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
    def arcadeDrive(self, rawStickX, rawStickY, invertX = False, invertY = True, swapVW = False, swapRL = False):
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
