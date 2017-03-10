import pigpio
import wavePWM
import pygame
from time import sleep

pygame.joystick.init()
pygame.init()

j = pygame.joystick.Joystick(0)
j.init()

pi = pigpio.pi()
pwm = wavePWM.PWM(pi)

pwm.set_cycle_time(3000)

def math(val):
	return (val + 3) * 500

try:
	while True:
		pygame.event.get()
		pwm.set_pulse_length_in_micros(6, math(j.get_axis(1)))
		pwm.update()
		sleep(0.1)
except KeyboardInterrupt:
	pwm.cancel()
	pi.stop()
