#!/usr/bin/python
__author__ = "Lentin Joseph"

'''
This node will train an SVM from a CSV file
and classify the values from virtual sensor
'''
import numpy as np
import pandas as pd
import csv
from sensor_msgs.msg import Image
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score
from sklearn.metrics import confusion_matrix
import pickle
import time
import cv2

from cv_bridge import CvBridge, CvBridgeError
import rospy
from std_msgs.msg import Int32

import sys
np.set_printoptions(threshold=sys.maxsize)



rfm_model = pickle.load(open("/home/tb/gazebo_road_generation/ros_ws/src/ScikitLearnSVM/rfm_model/rfm_py27.pickle", 'rb'))

def imageCb(data):
	try:
	 cv_image = CvBridge().imgmsg_to_cv2(data, "mono8")
	except CvBridgeError as e:
	 print(e)
   
	cv_image = cv2.erode(cv_image,(5,5),iterations = 2)

	cv2.imwrite("img.png", cv_image)

	flattened =  cv_image.reshape(1,-1)
	
		    
	prediction = rfm_model.predict(flattened)
	
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


