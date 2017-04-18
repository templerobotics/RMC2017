from Phidgets.Devices.Stepper import Stepper
from ..global_constants import TED_STEPPER_INDEX

controller = Stepper()
controller.openPhidget()

#Converts a number of steps into the equivalent degrees
def step2deg(steps, asBearing = False):
    #If asBearing is True, this returns the bearing in degrees of the given step
    #position
    steps = steps % 400 if asBearing else steps
    return float(steps * 0.9)

#Converts degrees to the nearest amount of steps needed
#Also returns the actual degrees that the steps represent, since it is rounded
def deg2step(degrees, asPosition = True):
    #This is very important, as not only does it simplify the math, but it also
    #prevents cable wrapping
    degrees = degrees % 360 if asPosition else degrees
    steps = int(round(degrees / 0.9))
    return steps, step2deg(steps)

class CameraStepper:

    def __init__(self, index = TED_STEPPER_INDEX):
        self.index = index

        controller.setEngaged(self.index, True)

        self.engaged = True

        self.zero()

        controller.setPositionMax(self.index, 399)
        controller.setPositionMin(self.index, 0)

    def __del__(self):
        controller.setEngaged(self.index, False)

    #Engages the motor; this means it can be given commands and will hold its
    #position
    def engage(self):
        if not controller.getEngaged(self.index):
            controller.setEngaged(self.index, True)
            self.engaged = True

    #Redirects to engage()
    def reengage(self):
        self.engage()

    #Disengages the motor; it will not respond to commands and will not hold its
    #position
    def disengage(self):
        if controller.getEngaged(self.index):
            controller.setEngaged(self.index, False)
            self.engaged = False

    #Turns the stepper to a given bearing.
    #Note: it will NOT wrap around, so if it is currently at 359deg and you send
    #it to 0deg, it will travel 359deg instead of 1deg
    def goto(self, angle):
        if self.engaged:
            steps, actualAngle = deg2step(angle, True)
            controller.setTargetPosition(self.index, steps)
            return actualAngle

    #If true, the motor is moving, if false, it is likely at its destination
    def isMoving(self):
        return not controller.getStopped(self.index)

    def stop(self):
        controller.setTargetPosition(self.index, controller.getPosition(self.index))
