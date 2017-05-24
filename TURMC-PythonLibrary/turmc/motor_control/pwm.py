import pigpio
import wavePWM
import subprocess
from ..global_constants import PIGPIO_PI_REFERENCE as PIGPIO_DAEMON
from time import sleep

#Initialize a global referencing the pigpio daemon
#PIGPIO_DAEMON = pigpio.pi() #Now imported

#Legacy handle for SabretoothLinearActuator, which used to be LinearActuator
def LinearActuator(pin):
    return SabretoothLinearActuator(pin)

#A generic PWM generator object. Useful for testing
class Wave:

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

    #Cleanly ends the wave
    def __del__(self):
        self._wave.cancel()

    #Irreversibly stops this wave
    def stop(self):
        del self

    #Sets the frequency in Hertz
    def setFrequency(self, frequency):
        self._wave.set_frequency(frequency)
        self._wave.update()

        self._frequency = frequency
        self._cycleTime = 1000000.0 / self._frequency

        self.setDutyCycle(self.getDutyCycle())

    #Sets the cycle time in microseconds
    def setCycleTime(self, time):
        self._wave.set_cycle_time(time)
        self._wave.update()

        self._cycleTime = time
        self._frequency = 1000000.0 / self._cycleTime

        self.setDutyCycle(self.getDutyCycle())

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
                self.setSpeed(self.getSpeed())
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

#For controlling stepper motors using the STM-5404 microstep driver
class MicrostepStepperMotor:

    #stepPin and dirPin are BCM/GPIO pin numbers
    def __init__(self, stepPin, dirPin, drillUpDirection = 1):
        self._stepPin = stepPin
        self._dirPin = dirPin

        self._upDirection = drillUpDirection
        self._downDirection = 0 if drillUpDirection == 1 else 1
        self.currentDirection = 0

        PIGPIO_DAEMON.write(self._dirPin, self.currentDirection)

        self._wave = wavePWM.PWM(PIGPIO_DAEMON)

        self._isSuspended = False

        self.setDirection(self._upDirection)

    #Safely cancels the pwm wave before deletion
    def __del__(self):
        self._wave.cancel()

    #Added for ease of programming
    def _update(self):
        self._wave.set_pulse_length_in_fraction(self._stepPin, 0.5)
        self._wave.update()

    #Deactivates the stepper motor pwm, which should halt the motor
    def suspend(self):
        self._wave.cancel()
        self._isSuspended = True

    #Reinitializes the pwm object without restarting the pwm wave
    def resume(self):
        self._wave = wavePWM.PWM(PIGPIO_DAEMON)
        self._isSuspended = False

    #Stops the motor by suspending it
    def stop(self):
        if not self._isSuspended:
            self.suspend()

    def hold(self):
        if not self._isSuspended:
            self._wave.set_cycle_time(600000000) #This sets the period to 10 minutes

    #Sets the pace in seconds per inch of the stepper. Assumes microsteps/revolution is 10000
    def setPace(self, pace):
        if self._isSuspended:
            self.resume()
        time = pace * 10
        self._wave.set_cycle_time(time)
        self._update()

    def setDirection(self, direction, stopOnDirectionChange = True):
        if not self.currentDirection == direction:
            if stopOnDirectionChange:
                self.stop()
            PIGPIO_DAEMON.write(self._dirPin, direction)
            self.currentDirection = direction

    def getDirection(self):
        if self.currentDirection == self._upDirection:
            return 'up'
        else:
            return 'down'

    def reverse(self):
        direction = self._upDirection if self.currentDirection == self._downDirection else self._downDirection
        self.setDirection(direction)

    def up(self):
        if self.currentDirection == self._downDirection:
            self.setDirection(self._upDirection)

    def down(self):
        if self.currentDirection == self._upDirection:
            self.setDirection(self._downDirection)

class ParallaxServoMotor:

    def __init__(self, pin):

        self._pin = pin

        self._TIME0 = 5000
        self._TIME180 = 2500

        self._wave = wavePWM.PWM(PIGPIO_DAEMON)

        self._wave.set_cycle_time(20000)

        self._currentPulseWidth = -1

        #self.setPosition(90)
        self._isSuspended = False

    #Makes sure to stop the motor and cancel the PWM before garbage collection
    def __del__(self):
        self.suspend()

    def _degreesToTime(self, degrees):
        return (float(degrees / 180.0) * (self._TIME180 - self._TIME0)) + self._TIME0

    def _timeToDegrees(self, time):
        return float((time - self._TIME0) / (self._TIME180 - self._TIME0)) * 180.0

    #Stops the calling object from generating PPM signals. Stopping safely means setting the position to 90 before suspending
    def suspend(self, safely = True):
        if not self._isSuspended:
            if safely:
                self.setPosition(90)
            self._wave.cancel()
            self._isSuspended = True

    #Reinitializes the motor object after having been suspended
    def resume(self, resumePosition = False):
        if self._isSuspended:
            self._wave = wavePWM.PWM(PIGPIO_DAEMON)
            self._wave.set_cycle_time(20000)
            self._isSuspended = False
            if resumePosition:
                self.setPosition(self.getPosition())
            else:
                self.setPosition(90)

    def setPosition(self, degrees):
        if self._isSuspended:
            self.resume()

        if degrees < 0:
            degrees = 0
        elif degrees > 180:
            degrees = 180

        self._currentPulseWidth = self._degreesToTime(degrees)

        self._wave.set_pulse_length_in_micros(self._pin, self._currentPulseWidth)
        self._wave.update()

        return degrees

    def getPosition(self):
        return self._timeToDegrees(self._currentPulseWidth())

class HiTecServoMotor:

    def __init__(self, pin):

        self._pin = pin

        self._TIME0 = 553
        self._TIME180 = 2520

        self._wave = wavePWM.PWM(PIGPIO_DAEMON)

        self._wave.set_cycle_time(4000)

        self._currentPulseWidth = -1

        #self.setPosition(90)
        self._isSuspended = False

    #Makes sure to stop the motor and cancel the PWM before garbage collection
    def __del__(self):
        self.suspend()

    def _degreesToTime(self, degrees):
        return (float(degrees / 180.0) * (self._TIME180 - self._TIME0)) + self._TIME0

    def _timeToDegrees(self, time):
        return float((time - self._TIME0) / (self._TIME180 - self._TIME0)) * 180.0

    #Stops the calling object from generating PPM signals. Stopping safely means setting the position to 90 before suspending
    def suspend(self, safely = True):
        if not self._isSuspended:
            if safely:
                self.setPosition(90)
            self._wave.cancel()
            self._isSuspended = True

    #Reinitializes the motor object after having been suspended
    def resume(self, resumePosition = False):
        if self._isSuspended:
            self._wave = wavePWM.PWM(PIGPIO_DAEMON)
            self._wave.set_cycle_time(20000)
            self._isSuspended = False
            if resumePosition:
                self.setPosition(self.getPosition())
            else:
                self.setPosition(90)

    def setPosition(self, degrees):
        if self._isSuspended:
            self.resume()

        if degrees < 0:
            degrees = 0
        elif degrees > 180:
            degrees = 180

        self._currentPulseWidth = self._degreesToTime(degrees)

        self._wave.set_pulse_length_in_micros(self._pin, self._currentPulseWidth)
        self._wave.update()

        return degrees

    def getPosition(self):
        return self._timeToDegrees(self._currentPulseWidth())

class VexMotorModule:

    def __init__(self, pin):

        self._pin = pin

        self._wave = wavePWM.PWM(PIGPIO_DAEMON)

        self._wave.set_cycle_time(20000)

        self._currentPulseWidth = -1

        self.setSpeed(0.0)
        self._isSuspended = False

    #Makes sure to stop the motor and cancel the PWM before garbage collection
    def __del__(self):
        self.suspend()

    def _speedToTime(self, speed):
        return (float(speed / 2.0) + 1.5) * 1000.0

    def _timeToSpeed(self, time):
        return (float(time / 1000.0) * 1.5) - 2.0

    #Stops the calling object from generating PPM signals. Stopping safely means setting the position to 90 before suspending
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
            self._wave.set_cycle_time(20000)
            self._isSuspended = False
            if resumeSpeed:
                self.setSpeed(self.getSpeed())
            else:
                self.setSpeed(0.0)

    def setSpeed(self, speed):
        if self._isSuspended:
            self.resume()

        if speed < -1.0:
            speed = -1.0
        elif speed > 1.0:
            speed = 1.0

        self._currentPulseWidth = self._speedToTime(speed)

        self._wave.set_pulse_length_in_micros(self._pin, self._currentPulseWidth)
        self._wave.update()

        return speed

    def getSpeed(self):
        return self._timeToSpeed(self._currentPulseWidth())

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

    #Inverts the directions of a specific actuator
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
