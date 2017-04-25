import time
import pigpio
import encoder_class

#encoder tick position
pos = 0
#save encoder data for furthur analysis
debug = [0.000]

#Encoder-Specfic Variables
#Cycles per revolution
CPR = 100
cycles = CPR / 360.00 #One complete quadurate rotation cycle

def writefile(list):
    for item in list:
      thefile.write("%s\n" % item)


def callback(way):
    global pos
    pos += way
    debug.append(way)
    print("pos={}".format(pos))
    #for debugging purposes
    if pos == 100:
        print("1 Full shaft rotation completed")


#Start pi object, will eventually need to integrate this with the MotorControl Library's Pi instance from Pigpio
pi = pigpio.pi()

decoder = encoder_class.decoder(pi,14,4,callback)
time.sleep(300)
print("Writing Debug")
writefile(debug)
decoder.cancel()
pi.stop()





