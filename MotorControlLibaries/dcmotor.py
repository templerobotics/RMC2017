import time
import RPi.GPIO as GPIO

class DCmotor:
	def __init__(self, setmode, setwarnings, pin1, pin2):
		self.setmode = GPIO.setmode(GPIO.BCM)
		self.setwarnings = GPI0.setwarnings(0)
		self.pin1 = GPIO.setup('''num''',GPIO.OUT)
		self.pin2 = GPIO.setup('''num''',GPIO.OUT)
	
	def dcmotorrun(self):
		while True: 
			try:
				self.pin1 = GPIO.output('''num''',1)
				self.pin2 = GPIO.output('''num''',0)
			except(KeyboardInterrupt):
				self.pin1 = GPIO.output('''num''',0)
				self.pin2 = GPIO.output('''num''',0)
				print("Stopping.")
				quit()
		

D = DCmotor()
D.dcmotorrun()

