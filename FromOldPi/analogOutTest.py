import pigpio
import wavePWM
from time import sleep

pi = pigpio.pi()

pwm = wavePWM.PWM(pi)

pwm.set_frequency(1000)

pwm.set_pulse_length_in_fraction(12, 0.0)

pwm.update()

def wave(direction):
	if direction == 'up':
		r = range(1, 100, 1)
	else:
		r = range(99, 0, -1)

	for n in r:
		pwm.set_pulse_length_in_fraction(12, float(n / 100.0))
		pwm.update()
		print(n)
		sleep(0.01)

	return 'yay'

while True:
	wave('up')
	wave('down')
