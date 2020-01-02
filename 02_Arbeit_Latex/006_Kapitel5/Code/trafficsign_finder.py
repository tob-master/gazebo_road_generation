#!/usr/bin/env python
from __future__ import print_function
import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from python_machine_learning_tutorials.msg import IntList
from python_machine_learning_tutorials.msg import Sign
from timeit import default_timer as timer




stop_cascade = cv2.CascadeClassifier('/home/tb/car_controle/HaarCascades/stop_sign.xml')
yield_cascade = cv2.CascadeClassifier('/home/tb/car_controle/HaarCascades/vorfahrt_gew_15stages_20wh.xml')
er_cascade = cv2.CascadeClassifier('/home/tb/car_controle/HaarCascades/same.xml')
#

class image_converter:

  def __init__(self):

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/image",Image,self.callback)
    self.pub=rospy.Publisher('/traffic_sign_finder',Sign,queue_size = 1)
    self.traffic_signs = Sign()  
    self.a = 0

  def callback(self,data):
    start = timer()
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
   

 #   gray=cv2.flip(gray,1)
        # flip image sideways
   # gray=cv2.flip(gray,0)

 #   cv_image=cv2.flip(cv_image,1)
        # flip image sideways
  #  cv_image=cv2.flip(cv_image,0)


    
    yield_sign = yield_cascade.detectMultiScale(gray,2,2)

    stop_sign = stop_cascade.detectMultiScale(gray,1.1,4)
    #er_sign = er_cascade.detectMultiScale(gray,1.1,5)
    

 #   if len(er_sign) < 1:
#	self.traffic_signs.Vel30 = False

  #  if len(yield_sign) < 1:
#	self.traffic_signs.Stop = False

  #  if len(stop_sign) < 1:
#	self.traffic_signs.Yield = False

		

   # for (x,y,w,h) in er_sign:
    #    cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2)
    #    roi_gray = gray[y:y+h, x:x+w]
     #   roi_color = cv_image[y:y+h, x:x+w]
	#self.traffic_signs.Vel30 = True



    for (x,y,w,h) in stop_sign:
        cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,255,0),2)
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = cv_image[y:y+h, x:x+w]
	self.traffic_signs.Stop = True



    for (x,y,w,h) in yield_sign:
        cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,0,255),2)
        roi_gray = gray[y:y+h, x:x+w]
        roi_color = cv_image[y:y+h, x:x+w]
	self.traffic_signs.Yield = True

    self.pub.publish(self.traffic_signs)
   
    cv2.imshow("Image window", cv_image)

    end = timer()
    print(end - start)
    cv2.waitKey(3)

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
 
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

