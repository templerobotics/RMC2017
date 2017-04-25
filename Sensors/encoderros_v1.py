import time
import pigpio
import encoder_class

#encoder tick position
pos = 0


def callback(way):
    global pos
    pos = way
    print("pos={}".format(pos))


#Start pi object, will eventually need to integrate this with the MotorControl Library's Pi instance from Pigpio
pi = pigpio.pi()

decoder = encoder_class.decoder(pi,14,4,callback)
time.sleep(300)
decoder.cancel()
pi.stop()





