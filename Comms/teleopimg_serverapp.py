import cv2
from socket import *
from matplotlib import pyplot as plt
#Program for recieving requested images from Intel Nuc


#recieving port
port_recv = 16000
recvSocket = socket(AF_INET, SOCK_DGRAM)
recvSocket.bind(('', port_recv))


while True:
    img, address = recvSocket.recvfrom(16000)
    print(address)
    cv2.imshow('img', img)
    cv2.waitKey(0)
