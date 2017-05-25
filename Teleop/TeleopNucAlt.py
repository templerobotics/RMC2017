from turmc.global_constants import *
from turmc.networking.joystick import Server
from turmc.networking.image_comm import ImageSender
#from turmc.motor_control.camera_stepper import CameraStepper

BILL = cv2.VideoCapture(0)
TED = cv2.VideoCapture(1)

#stepper = CameraStepper()

imageSender = ImageSender('192.168.1.105', BASE_COMPUTER_PORT)

def sendImage(cameraName, grayscale = True, metadata = {}):
    if cameraName == 'Bill':
        _, frame = BILL.read()
    elif cameraName == 'Ted':
        _, frame = TED.read()
    else:
        return
    if grayscale:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    imageSender.sendImage(frame)

def sendImages():
    if sendTed or sendBill:
        if sendTed:
            _, frame = TED.read()
        elif sendBill:
            _, frame = BILL.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        imageSender.sendImage(frame)


def updateFlags(data):
    global sendBill, sendTed
    if data['button11'] == 1.0:
        sendBill = True
        sendTed = False
    elif data['button12'] == 1.0:
        sendTed = True
        sendBill = False
    else:
        sendTed = False
        sendBill = False

    #Camera stepper controls; will hold position if not moving
    # if data['hatX'] == -1:
    #     stepper.stepLeft()
    # elif data['hatX'] == 1:
    #     stepper.stepRight()
    # else:
    #     stepper.stop()

def main():
    global tServer, sendTed, sendBill
    tServer = TimeoutServer(BASE_COMPUTER_PORT, updateFlags, sendImage, 1.0)

    sendTed = False
    sendBill = False

    tServer.start()

if __name__ == '__main__':
    main()
