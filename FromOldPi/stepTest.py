#!/usr/bin/env python

import pigpio
import wavePWM

pi = pigpio.pi()

pi.set_mode(12, pigpio.OUTPUT)
pi.set_mode(6, pigpio.OUTPUT)

pi.write(DIR, pigpio.LOW)


