import cv2
from socket import *
from matplotlib import pyplot as plt
import numpy as np
#Program for recieving requested images from Intel Nuc



#recieving port
port_recv = 16000
recvSocket = socket(AF_INET, SOCK_DGRAM)
recvSocket.bind(('', port_recv))

while True:
    data, address = recvSocket.r
    #img = cv2.imread(img)
    print(address)
    frame = np.fromstring(data, dtype=np.uint8)
    print frame.shape
    frame = np.reshape(frame, (480,128))
    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    #plt.imshow(frame)
    #plt.show()
    #cv2.imshow('img', frame)
    #cv2.waitKey(25)