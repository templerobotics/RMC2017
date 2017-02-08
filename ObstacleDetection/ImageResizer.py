#!/usr/bin/env python

#Quick script to grab positive images and resize them, and then go onto
#grabbing negative images online and also editing them



import cv2
import urlib.request as urlib
import numpy as np
import os


class ImagePrep:
    #Use to resize collected positive images
    def pos_images():
        
        #Edit this for new path of positive imges
    
        pos_path = '/path/folder'

        files = [f for f in listdir(pos_path) if isfile(join(pos_path,f)) ]

        #empty array with the size of the amount of files we have
        images = numpy.empty(len(files), dtype=object)
    
        pos_num = 1


        #cycle throw positives
        for n in range(0, len(files)):
          img[n] = cv2.imread( join(pos_path,files[n]),cv2.IMREAD_GRAYSCALE)
          img_resize = cv2.resize(img[n], (45, 45))
          cv2.imwrite("pos/"+str(pos_num)+".jpg",img_resize)
          pos_num+=1
  
  
    #Use to pull and resize negative images from image-net
    def store_neg_images():
        neg_images_link = 'image-net url for negative images'   
        neg_image_urls = urllib.request.urlopen(neg_images_link).read().decode()
        #pic_num stands for picture index on the repo
        pic_num = 1
    
        if not os.path.exists('neg'):
            os.makedirs('neg')
        
        for i in neg_image_urls.split('\n'):
            try:
                print(i)
                urllib.request.urlretrieve(i, "neg/"+str(pic_num)+".jpg")
                neg_img = cv2.imread("neg/"+str(pic_num)+".jpg",cv2.IMREAD_GRAYSCALE)
                # should be larger than samples / pos pic (so we can place our image on it)
                neg_resize = cv2.resize(img, (100, 100))
                cv2.imwrite("neg/"+str(pic_num)+".jpg",neg_resize)
                pic_num += 1
                
            except Exception as e:
                print(str(e))  
  
      
            
    def create_bg():
        for file_type in ['neg']:
        
            for img in os.listdir(file_type):
    
                if file_type == 'pos':
                    line = file_type+'/'+img+' 1 0 0 50 50\n'
                    with open('info.dat','a') as f:
                        f.write(line)
                elif file_type == 'neg':
                    line = file_type+'/'+img+'\n'
                    with open('bg.txt','a') as f:
                        f.write(line)
  
  
  
  
  
  
  
  
                          
         
