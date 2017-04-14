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
    fig = plt.figure()
    fig.clear()
    data, address = recvSocket.recvfrom(16000)
    #img = cv2.imread(img)
    print(address)
    print(frame.shape)
    frame = np.fromstring(data, dtype=np.uint8).reshape(480)
    print frame.shape
    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    plt.imshow(frame)
    plt.show()
    plt.close()
    #cv2.imshow('img', frame)
    #cv2.waitKey(25)