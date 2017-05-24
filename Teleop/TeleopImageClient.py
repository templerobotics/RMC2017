import cv2
from turmc.global_constants import *
from turmc.networking.image_comm import ImageServer

cameras = ['Useless', 'Bill', 'Ted']

for cam in cameras:
    cv2.namedWindow(cam, cv2.WINDOW_NORMAL)

def callback(name, image):
    if name in cameras:
        cv2.imshow(name, image)
    if cv2.waitKey(30) == 27:
        return

def main():
    server = ImageServer(BASE_COMPUTER_PORT, callback)
    server.start()

#Boilerplate
if __name__ == '__main__':
    main()
