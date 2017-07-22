from turmc.global_constants import *
from turmc.motor_control.pwm import MicrostepStepperMotor
from turmc.motor_control.sabertooth_serial import DCMotor, LinearActuator, Drivetrain
from turmc.networking.joystick import JSONServer
from time import sleep

#Define all the motor objects
drivetrain = Drivetrain(128)
armActuator = LinearActuator(129, 1)
bucketActuators = LinearActuator(129, 2)

#Callback which handles joystick data from the server
def handle(data):
    #Updates the drivetrain with the joystick axial input
    if 'x' in data and 'y' in data:
        drivetrain.drive(data['x'], data['y'])

    #Arm actuator controls; retracts if both are pressed
    if 'arm' in data:
        if data['arm'] == -1.0:
            drillActuator.retract()
        elif data['arm'] == 1.0:
            drillActuator.extend()
        else:
            drillActuator.stop()

    #Bucket actuator controls; retracts if both are pressed
    if 'bucket' in data:
        if data['bucket'] == -1.0:
            conveyorActuator.retract()
        elif data['bucket'] == 1.0:
            conveyorActuator.extend()
        else:
            conveyorActuator.stop()

#Starts up a server listening on the RPi port
def main():
    server = JSONServer(RASPBERRY_PI_PORT, handle)
    server.start()

#Boilerplate
if __name__ == '__main__':
    main()
