#! /usr/bin/env python





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

    #Stops PWM generation to prevent motor twitching. Use when the robot needs to sit still
    def suspend(self):
        if not self._isSuspended:
            self._wave.cancel()
            self._wave.update()
            self._isSuspended = True

    #Resumes PWM generation so that the motors can be used
    def resume(self):
        if self._isSuspended:
            self._wave = wavePWM.PWM(self._pi)
            self._wave.set_cycle_time(3000)
            self._wave.set_pulse_length_in_micros(self._pin, 1500)
            self._wave.update()
            self._isSuspended = False

    #Takes a value from -1.0 to 1.0 to set the speed of the motor
    def updateSpeed(self, speed):
        if self._isSuspended:
            self.resume()

        if speed < -1.0:
            speed = -1.0
        elif speed > 1.0:
            speed = 1.0

        speed = (speed + 3.0) * 1000.0

        self._wave.set_pulse_length_in_micros(self._pin, speed)
        self._wave.update()

        self._currentPulseWidth = speed

        return speed

    #Runs a gradient from the current speed to a target speed. Number of steps and time of gradient are customizable
    def autoGradient(self, targetSpeed = 0.0, steps = 20, time = 1.0):
        if self._isSuspended:
            self.resume()

        if targetSpeed < -1.0:
            targetSpeed = -1.0
        elif targetSpeed > 1.0:
            targetSpeed = 1.0

        targetSpeed = (targetSpeed + 3.0) * 1000.0

        gradient = range(self._currentPulseWidth, targetSpeed, (targetSpeed - self._currentPulseWidth) / steps).append(targetSpeed)

        for speed in gradient:
            self._wave.set_pulse_length_in_micros(self._pin, speed)
            self._wave.update()
            sleep(float(time / steps))

        self._currentPulseWidth = targetSpeed

        return targetSpeed

    #Cancels the wave and then destroys this object
    def release(self):
        self._wave.cancel()
        del self

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
    def arcadeDrive(self, rawStickX, rawStickY, invertX = False, invertY = True):
        if invertX:
            rawStickX *= -1.0
        if invertY:
            rawStickY *= -1.0

        v = (1.0 - abs(rawStickX)) * rawStickY + rawStickY
        w = (1.0 - abs(rawStickY)) * rawStickX + rawStickX
        rSpeed = (v + w) / 2.0
        lSpeed = (v - w) / 2.0

        self._rightMotor.updateSpeed(rSpeed)
        self._leftMotor.updateSpeed(lSpeed)

        return (rSpeed, lSpeed)

    #Use for a dual joystick setup controlling a tank-style drivetrain
    def tankDrive(self, rawStickLeft, rawStickRight, invertLeft = False, invertRight = False):
        if invertLeft:
            rawStickLeft *= -1.0
        if invertRight:
            rawStickRight *= -1.0

        self._rightMotor.updateSpeed(rawStickRight)
        self._leftMotor.updateSpeed(rawStickLeft)

        return (rawStickRight, rawStickLeft)