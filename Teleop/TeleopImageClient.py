import cv2
from turmc.global_constants import *
from turmc.networking.image_comm import ImageServer
from matplotlib import pyplot as plt

cameras = ['Useless', 'Bill', 'Ted']


def construct_windows():
    global figure, Useless, Ted, Bill

    figure, (Useless, Ted, Bill)= plt.subplots(3, sharex=True)
    Useless.plot()
    Useless.set_title('useless')
    Useless.axis('off')
    
    Ted.plot()
    Ted.set_title('ted')
    Ted.axis('off')
    
    Bill.plot()
    Bill.set_title('bill')
    Bill.axis('off')

    plt.show('all')


def plotter(image):
    #global figure, useless, ted, bill

    img_handle = plt.imshow(image, cmap='gray')
    return img_handle
def callback(name, image):

    if name in cameras:
        img = plotter(image)
        name.plot(img)
        plt.show('all')
#    #if name in cameras:
#        #cv2.imshow(name, image)
#    #if cv2.waitKey(30) == 27:
#        #return


def main():
    construct_windows()
    server = ImageServer(BASE_COMPUTER_PORT, callback)
    server.start()


#Boilerplate
if __name__ == '__main__':
    main()
