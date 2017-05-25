import cv2
from turmc.global_constants import *
from turmc.networking.image_comm import ImageServer

def callback(image):
    if not image == None:
        cv2.imshow('Robot', image)
    if cv2.waitKey(30) == 27:
        return

def main():
    global server
    server = ImageServer(BASE_COMPUTER_PORT, callback)
    server.start()

#Boilerplate
if __name__ == '__main__':
    main()
