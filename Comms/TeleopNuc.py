from turmc.global_constants import *
from turmc.networking.joystick import Server

#The 'justSent' booleans make the images only get sent on presses, and prevents
#images from being spammed if someone holds down the button
justSentUseless = False
justSentBill = False
justSentTed = False

def handle(data):
    #Sends an image from Useless
    # if data['button10'] == 1.0 and not justSentUseless:
    #     sendImage('Useless')
    #     justSentUseless = True
    # elif data['button10'] == 0.0 and justSentUseless:
    #     justSentUseless = False

    #Sends an image from Bill
    if data['button11'] == 1.0 and not justSentBill:
        sendImage('Bill')
        justSentBill = True
    elif data['button11'] == 0.0 and justSentBill:
        justSentBill = False

    #Sends an image from Ted
    if data['button12'] == 1.0 and not justSentTed:
        sendImage('Ted')
        justSentTed = True
    elif data['button12'] == 0.0 and justSentTed:
        justSentTed = False

#Starts up a server listening on the Nuc port
def main():
    server = Server(NUC_PORT, handle)
    server.start()

#Boilerplate
if __name__ == '__main__':
    main()
