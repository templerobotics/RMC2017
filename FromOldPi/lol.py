#!/usr/bin/env python3

from sense_hat import SenseHat
from time import sleep
from random import *

s = SenseHat()

colors = range(0, 256)

def genColors():
	image = []
	for i in range(64):
		image.append((choice(colors), choice(colors), choice(colors)))
	return image

while True:
	s.set_pixels(genColors())
	sleep(0.5)
