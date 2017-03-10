import MotorControl as MC
import pygame
from time import sleep

pygame.joystick.init()
pygame.init()

j = pygame.joystick.Joystick(0)
j.init()

rm = MC.SabretoothDCMotor(6)
lm = MC.SabretoothDCMotor(18)

robot = MC.RobotDrive(rm, lm)

while True:
	pygame.event.get()
	robot.arcadeDrive(j.get_axis(0), j.get_axis(1), invertY = True)
	sleep(0.1)
