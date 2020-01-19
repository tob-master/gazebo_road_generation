#!/usr/bin/python

import keras
import tensorflow as tf
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten
from keras.layers import Conv2D, MaxPooling2D
from keras.models import model_from_json
#from keras import backend as K
#K.clear_session()
import numpy as np
import pandas as pd
import csv
from sensor_msgs.msg import Image
import pickle
import time
import cv2

from cv_bridge import CvBridge, CvBridgeError
import rospy
from std_msgs.msg import Int32

import sys
np.set_printoptions(threshold=sys.maxsize)
input_shape = (28,56,1)
num_classes = 9

json_file = open('/home/tb/gazebo_road_generation/ros_ws/src/ScikitLearnSVM/cnn_model/cnn_model.json', 'r')
loaded_model_json = json_file.read()
json_file.close()
loaded_model = model_from_json(loaded_model_json)
loaded_model.load_weights("/home/tb/gazebo_road_generation/ros_ws/src/ScikitLearnSVM/cnn_model/cnn_model.h5")
'''
loaded_model = Sequential()
loaded_model.add(Conv2D(32, kernel_size=(3, 3),
                 activation='relu',
                 input_shape=input_shape))
loaded_model.add(Conv2D(64, (3, 3), activation='relu'))
loaded_model.add(MaxPooling2D(pool_size=(2, 2)))
loaded_model.add(Dropout(0.25))
loaded_model.add(Flatten())
loaded_model.add(Dense(128, activation='relu'))
loaded_model.add(Dropout(0.5))
loaded_model.add(Dense(num_classes, activation='softmax'))
'''

loaded_model.compile(loss=keras.losses.categorical_crossentropy,optimizer=keras.optimizers.Adadelta(),metrics=['accuracy'])
#loaded_model.make_predict_function()
graph = tf.get_default_graph()

def imageCb(data):
	try:
	 cv_image = CvBridge().imgmsg_to_cv2(data, "mono8")
	except CvBridgeError as e:
	 print(e)
   
	#cv_image = cv2.erode(cv_image,(5,5),iterations = 2)


	ret,thresh1 = cv2.threshold(cv_image,200,1,cv2.THRESH_BINARY)

	#print(thresh1)

	#cv2.imwrite("img.png", thresh1*255)

	#flattened =  thresh1.reshape(1,-1)
	
	thresh_3d = np.expand_dims(thresh1, axis=0)
	thresh_4d = np.expand_dims(thresh_3d, axis=4)
	#print(thresh_4d.shape)
	
	global graph
	with graph.as_default():	    
		prediction = loaded_model.predict_classes(thresh_4d)
	
	pred_str = str(prediction[0]+1) + "0 km/h"	

	#prediction = rfm_model.predict_proba(flattened)

	print(pred_str)
	#print(flattened.shape)

	#cv2.imwrite("thresh.png", cv_image)
       	#print(cv_image)
	
	#im_gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
	#im_gray = cv2.GaussianBlur(cv_image, (5, 5), 0)
	
	#erosion = cv2.erode(cv_image,(5,5),iterations = 5)

	#cv2.imwrite("gray.png", erosion)
	#ret, im_th = cv2.threshold(erosion, 90, 255, cv2.THRESH_BINARY)

	#cv2.imwrite("th.png", im_th)
	#_,ctrs, hier = cv2.findContours(im_th.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	#cv2.drawContours(im_th, ctrs, -1, (0,255,0), cv2.LINE_AA)
	

	#rects = [cv2.boundingRect(ctr) for ctr in ctrs]

	#print(rects)

	#count = 0
	'''
	for rect in rects:
	    #cv2.imwrite("thresh.png", im_th) 

	    #cv2.rectangle(im, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 255, 0), 3) 
	    #cv2.imwrite("rect.png", im) 
	    

	    roi = im_th[rect[1]-2:int((rect[1]+rect[3])+2),rect[0]-2:int((rect[0]+rect[2])+2)]
	    

	    #cv2.imwrite("cropped.png", roi) 
	    
	    roi = cv2.resize(roi, (28, 56), interpolation=cv2.INTER_AREA)
	    
	    name = "img_" + str(count) + ".png"	    

	    cv2.imwrite(name, roi)			
	    count +=1		
	    data = np.array(roi)
	    flattened = data.flatten()
	    

	    #cv2.imwrite("resized.png", roi)
	    #roi = cv2.dilate(roi, (3, 3))
	    #cv2.imwrite("dilatet.png", roi)
	    

	    flattened2d =  flattened.reshape(1,-1)
	    
	    nbr = rfm_model.predict(flattened2d)
	    print(nbr[0])
	'''

def listener():

	global obj
	global pub

	rospy.loginfo("Starting prediction node")

	rospy.init_node('listener', anonymous = True)

	#rospy.Subscriber("sensor_read", Int32, read_sensor_data)

	image_sub = rospy.Subscriber("road_sign/image",Image, imageCb)

	#pub = rospy.Publisher('predict', Int32, queue_size=1)


	#im = cv2.imread("/home/tb/gazebo_road_generation/ros_ws/src/ScikitLearnSVM/rfm_model/5.jpg")



	#print("ok")


	rospy.spin()

if __name__ == '__main__':
	listener()


