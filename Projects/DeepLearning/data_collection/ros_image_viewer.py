#!/usr/bin/env python
print("\nEnsure roscore is running")
import rospy
import os
import time
import sys
import numpy as np

from matplotlib import pyplot as plt
from matplotlib.patches import Circle
from matplotlib.patches import Rectangle
import matplotlib.image as mpimg

#Used to multi task
import threading
from threading import Thread

# ROS Image message
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

bridge = CvBridge()

# lower_red = np.array([140,30,30])
# upper_red = np.array([179,255,255])

# lower_blue = np.array([108,60,60])
# upper_blue = np.array([113,120,120])

# lower_yellow = np.array([12,30,30])
# upper_yellow = np.array([26,255,255])

x_min = 100
x_max = 550
y_min = 100
y_max = 250

mask_plotting = True

lower_red = np.array([140,120,60])
upper_red = np.array([176,255,200])

lower_blue = np.array([107,170,60])
upper_blue = np.array([115,220,225])

lower_yellow = np.array([12,30,30])
upper_yellow = np.array([26,255,255])

lower_green = np.array([74,30,30])
upper_green = np.array([85,255,255])

lower_gray = np.array([108,60,60])
upper_gray = np.array([113,120,120])

lower_threshold = {"red": lower_red,"blue": lower_blue, "yellow": lower_yellow, "green": lower_green, "gray": lower_gray}
upper_threshold = {"red": upper_red,"blue": upper_blue, "yellow": upper_yellow, "green": upper_green, "gray": upper_gray}

color_codes = {"red": (0,0,255), "blue": (255,0,0), "yellow": (0,255,255), "green": (0, 255, 0), "gray": (192,192,192)}

classified_colors_list = []
times = []

def intro():
	image_topic = raw_input("\n\nName of the topic which the Images are Published (ex: topic_name/sub_topic): \n")
	image_type = raw_input("\n\nWill you be subscribing to images of message type Image (i) or CompressedImage (c)  (i/c):\n")
	if image_type == 'c': image_type = 'CompressedImage'
	else: image_type = 'Image'
	print(chr(27) + "[2J")
	print "Images will be subscribed from topic ", image_topic, "\nwith message type: ", image_type, "\n\n... Is this correct? (y/n)"
	confirmation = raw_input("")

	if confirmation != "y":
		print("Please Restart the script and try entering the information correctly")
		assert confirmation == "y", "image topic and number not properly specified"

	print(chr(27) + "[2J")
			
	print "\n\nA new window will open displaying the images from topic:", image_topic, "\n\n"

	return image_topic, image_type

class image_converter:

  global image_topic
  global image_type
  def __init__(self):
    # self.image_pub = rospy.Publisher("image_topic_2",Image)
    print("Initializing Color Recognition \nA new window will appear once image are detected ...")
    self.bridge = CvBridge()
    # self.image_sub = rospy.Subscriber("camera1/usb_cam/image_raw/compressed",CompressedImage,self.callback, queue_size=1)
    if image_type == "Image":
	    self.image_sub = rospy.Subscriber(image_topic,Image,self.callback, queue_size=1)
    else:
	    self.image_sub = rospy.Subscriber(image_topic,CompressedImage,self.callback, queue_size=1)

  def callback(self,data):
  	global image_type
	try:

	  if image_type == "Image": 
		  cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	  else:	
		  cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")

	  cv2.namedWindow('Image',cv2.WINDOW_NORMAL)
	  #cv2.moveWindow('Image', 40,100)
	  cv2.resizeWindow('Image', 700,450)
	  cv2.imshow("Image", cv_image)
	  cv2.waitKey(1)

	except CvBridgeError as e:
	  print(e)
	

def main(args):

	global image_topic
	global image_type

	image_topic, image_type = intro()

	ic = image_converter()
	rospy.init_node('image_converter', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)