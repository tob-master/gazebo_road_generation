from PIL import Image
import cv2
import numpy as np
import os

7


def convert_to_grayscale():
    match = False
    for file_type in ['Traffic_Signs/00001/']:
        for img in os.listdir(file_type):
                    current_image_path = str(file_type)+'/'+str(img)
                    gray = cv2.imread(current_image_path,cv2.CV_LOAD_IMAGE_GRAYSCALE)
	            img = str(img)
		    img = img[:-3] + 'jpg'
		    
                    cv2.imwrite('pos_img/'+str(img) , gray)


convert_to_grayscale()
		    


           
