import pigpio
import wavePWM

pi = pigpio.pi()
pwm = wavePWM.PWM(pi)

pi.write(12, 0)

pwm.set_cycle_time(20)
pwm.set_pulse_length_in_micros(6, 10)
pwm.update()
