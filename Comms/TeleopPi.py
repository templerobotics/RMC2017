from turmc.global_constants import *
from turmc.motor_control.sabertooth_serial import DCMotor, LinearActuator, Drivetrain
from turmc.networking.joystick import Server

#Define all the motor objects
drivetrain = Drivetrain(SERIAL_ADDR_DRIVETRAIN)
drillActuator = LinearActuator(SERIAL_ADDR_LINEAR_ACUATORS, MOTOR_NUMBER_DRILL_ACTUATOR)
conveyorActuator = LinearActuator(SERIAL_ADDR_LINEAR_ACUATORS, MOTOR_NUMBER_CONVEYOR_ACTUATOR)
auger = DCMotor(SERIAL_ADDR_OTHER_MOTORS, MOTOR_NUMBER_AUGER)
conveyorMotor = DCMotor(SERIAL_ADDR_OTHER_MOTORS, MOTOR_NUMBER_CONVEYOR) #DO NOT EXCEED SPEED OF 0.5 FOR THIS MOTOR

#Callback which handles joystick data from the server
def handle(data):
    #Updates the drivetrain with the joystick axial input
    drivetrain.drive(data['joyX'], data['joyY'])

    #Auger controls; full speed if both are pressed
    if data['button1'] = 1.0:
        auger.setSpeed(1.0)
    elif data['button7'] = 1.0:
        auger.setSpeed(-0.5)
    else:
        auger.stop()

    #Conveyor controls; full speed if both are pressed
    if data['button2'] = 1.0:
        conveyorMotor.setSpeed(1.0)
    elif data['button8'] = 1.0:
        conveyorMotor.setSpeed(-0.5)
    else:
        conveyorMotor.stop()

    #Drill actuator controls; retracts if both are pressed
    if data['button3'] = 1.0:
        drillActuator.retract()
    elif data['button5'] = 1.0:
        drillActuator.extend()
    else:
        drillActuator.stop()

    #Conveyor actuator controls; retracts if both are pressed
    if data['button4'] = 1.0:
        conveyorActuator.retract()
    elif data['button6'] = 1.0:
        conveyorActuator.extend()
    else:
        conveyorActuator.stop()

    #TODO: implement the drill track stepper into manual control

#Starts up a server listening on the RPi port
def main():
    server = Server(RASPBERRY_PI_PORT, handle)
    server.start()

#Boilerplate
if __name__ == '__main__':
    main()
