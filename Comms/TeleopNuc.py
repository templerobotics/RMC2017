from turmc.global_constants import *
from turmc.networking.joystick import Server
from turmc.networking.image_comm import ImageSender
from turmc.motor_control.camera_stepper import CameraStepper

#The 'justSent' booleans make the images only get sent on presses, and prevents
#images from being spammed if someone holds down the button
justSentUseless = False
justSentBill = False
justSentTed = False

#Initialize camera objects
TED = cv2.VideoCapture(TED_INDEX)
BILL = cv2.VideoCapture(BILL_INDEX)
USELESS = cv2.VideoCapture(USELESS_INDEX)

#Initialize the camera stepper object
stepper = CameraStepper()

#Creates an ImageSender object pointing at the base computer
imageSender = ImageSender(BASE_COMPUTER_IP, BASE_COMPUTER_PORT)

def sendImage(cameraName, grayscale = True, metadata = {}):
    if cameraName == 'Useless':
        frame = USELESS.read()
    elif cameraName == 'Bill':
        frame = BILL.read()
    elif cameraName == 'Ted':
        frame = TED.read()
    else:
        return
    if grayscale:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    imageSender.sendImage(frame, cameraName, metadata)

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
        sendImage('Ted', metadata = {'position': stepper.getPosition(), 'bearing': stepper.getBearing()})
        justSentTed = True
    elif data['button12'] == 0.0 and justSentTed:
        justSentTed = False

    #Camera stepper controls; will hold position if not moving
    if data['hatX'] == -1:
        stepper.stepLeft()
    elif data['hatX'] == 1:
        stepper.stepRight()
    else:
        stepper.stop()

#Starts up a server listening on the Nuc port
def main():
    server = Server(NUC_PORT, handle)
    server.start()

#Boilerplate
if __name__ == '__main__':
    main()
