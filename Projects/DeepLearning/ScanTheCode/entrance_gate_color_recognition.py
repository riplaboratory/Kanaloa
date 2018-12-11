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
from std_msgs.msg import String
from std_msgs.msg import Int16
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

bridge = CvBridge()

x_min = 400
x_max = 1000
y_min = 200
y_max = 450

min_radius = 10

mask_plotting = True


lower_red = np.array([0,90,120])
upper_red = np.array([5,230,200])

lower_green = np.array([40,50,50])
upper_green = np.array([72,210,210])

lower_threshold = {"red": lower_red, "green": lower_green}
upper_threshold = {"red": upper_red, "green": upper_green}

color_codes = {"red": (0,0,255), "green": (0, 255, 0)}

toss_images = 0
toss_images_threshold = 1


classified_colors_list = []
times = []

def intro():
	print("\nTopics:")
	topics = rospy.get_published_topics()
	i = 0
	topic_list = []
	type_list = []
	for topic in topics :
		topic_name = topic[0]
		topic_name = topic_name[1:]
		topic_type = topic[1]
		topic_type = topic_type.split("/", 1)
		#Sort Topics by type
		if topic_type[1] == "CompressedImage" or topic_type == "Image":
			print i, ") \t", topic_name
			topic_list.append(topic_name)
			type_list.append(topic_type[1])
			i += 1

	#Choosing a topic number
	a = int(raw_input("\n\nPick A topic number:  "))
	if a < 0 or a >= i:
		assert "Invalid Option"
	else:
		print topic_list[a], type_list[a] 

	image_topic = topic_list[a]
	image_type = type_list[a]

	print(chr(27) + "[2J")
			
	print "\n\nA new window will open displaying the images from topic:", image_topic, "\n\n"

	return image_topic, image_type

def scan_the_code(colors, times):
    diff = np.asarray([])
    colors = np.asarray(colors)
    times = np.asarray(times)
    for i in range(len(times)):
        if i == 0:
            diff = np.append(diff, 0)
        else:
            diff = np.append(diff, times[i] - times[i-1])

    diff_max = np.argmax(diff) #Returns position of highest value

    if len(colors)-3 < diff_max: #If at end of the list, count backwards to find code. 
        scan = [colors[diff_max], colors[diff_max-1], colors[diff_max-2]]
    else:
        scan = [colors[diff_max], colors[diff_max+1], colors[diff_max+2]]
        
    return scan

def plot_scan_the_code(scanned):
	scan_the_code_report_template = cv2.imread('images/scan_the_code_report_template.png')

	cv2.rectangle(scan_the_code_report_template, (40, 200), (285, 450), color_codes[scanned[0]], thickness = -1)
	cv2.rectangle(scan_the_code_report_template, (355, 200), (610, 450), color_codes[scanned[1]], thickness = -1)
	cv2.rectangle(scan_the_code_report_template, (680, 200), (950, 450), color_codes[scanned[2]], thickness = -1)

	cv2.putText(scan_the_code_report_template, scanned[0], (120,150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2);
	cv2.putText(scan_the_code_report_template, scanned[1], (450,150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2);
	cv2.putText(scan_the_code_report_template, scanned[2], (780,150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2);

	cv2.namedWindow('Scan the Code',cv2.WINDOW_NORMAL)
	cv2.moveWindow('Scan the Code', 40,650)
	cv2.resizeWindow('Scan the Code', 575,350)
	cv2.imshow("Scan the Code", scan_the_code_report_template)
	cv2.waitKey(1)

def plot_mask(mask, color, position):
	mask_name = str(color)+' Mask'
	cv2.namedWindow(mask_name,cv2.WINDOW_NORMAL)
	if position == 0:
		cv2.moveWindow(mask_name, 1150,30)
	elif position == 1:
		cv2.moveWindow(mask_name, 1150,310)
	elif position == 2:
		cv2.moveWindow(mask_name, 1150,590)
	else:
		cv2.moveWindow(mask_name, 1150,850)
	
	cv2.resizeWindow(mask_name, 600,250)
	cv2.imshow(mask_name, mask)
	cv2.waitKey(1)


def color_recognition(image):
    classified, coordinates, radius_dict, mask_dict, res_dict = {}, {}, {}, {}, {}
    
    global lower_threshold
    global upper_threshold
    
    for color in lower_threshold.keys():
        lower_color = lower_threshold[color]
        upper_color = upper_threshold[color]

        mask = cv2.inRange(image, lower_color, upper_color)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]
        res = cv2.bitwise_and(image, image, mask = mask)
        res_plot = cv2.cvtColor(res, cv2.COLOR_HSV2BGR)

        if mask_plotting:
        	if color == "red":
        		plot_mask(res_plot, color, 0)
    		if color == "green":
        		plot_mask(res_plot, color, 1)

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            if radius > min_radius: 

            	radius += 15
                classified[color] = True
                coordinates[color] = [x, y]
                radius_dict[color] = radius
                mask_dict[color] = mask
                res_dict[color] = res
            else: 
                classified[color] = False
        else: 
            classified[color] = False

    return classified, coordinates, radius_dict, mask_dict, res_dict
    # return classified, coordinates, radius_dict

class image_converter:

  global image_topic
  global image_type
  def __init__(self):

    self.pub_entrance_gate = rospy.Publisher('entrance_gate', String, queue_size=10)
    print("Initializing Color Recognition \nA new window will appear once image are detected ...")
    self.bridge = CvBridge()
    # self.image_sub = rospy.Subscriber("camera1/usb_cam/image_raw/compressed",CompressedImage,self.callback, queue_size=1)
    if image_type == "Image":
	    self.image_sub = rospy.Subscriber(image_topic,Image,self.callback, queue_size=1)
    else:
	    self.image_sub = rospy.Subscriber(image_topic,CompressedImage,self.callback, queue_size=1)
	    # self.test_sub = rospy.Subscriber("chatter", String, self.callback2, queue_size=1)
    self.sub_lower_r_h = rospy.Subscriber("entrance_gate/lower_threshold_r/hue", Int16, self.callback2, queue_size=1)
    self.sub_lower_r_s = rospy.Subscriber("entrance_gate/lower_threshold_r/saturation", Int16, self.callback3, queue_size=1)
    self.sub_lower_r_v = rospy.Subscriber("entrance_gate/lower_threshold_r/value", Int16, self.callback4, queue_size=1)
    self.sub_upper_r_h = rospy.Subscriber("entrance_gate/upper_threshold_r/hue", Int16, self.callback5, queue_size=1)
    self.sub_upper_r_s = rospy.Subscriber("entrance_gate/upper_threshold_r/saturation", Int16, self.callback6, queue_size=1)
    self.sub_upper_r_v = rospy.Subscriber("entrance_gate/upper_threshold_r/value", Int16, self.callback7, queue_size=1)

    self.sub_lower_g_h = rospy.Subscriber("entrance_gate/lower_threshold_g/hue", Int16, self.callback14, queue_size=1)
    self.sub_lower_g_s = rospy.Subscriber("entrance_gate/lower_threshold_g/saturation", Int16, self.callback15, queue_size=1)
    self.sub_lower_g_v = rospy.Subscriber("entrance_gate/lower_threshold_g/value", Int16, self.callback16, queue_size=1)
    self.sub_upper_g_h = rospy.Subscriber("entrance_gate/upper_threshold_g/hue", Int16, self.callback17, queue_size=1)
    self.sub_upper_g_s = rospy.Subscriber("entrance_gate/upper_threshold_g/saturation", Int16, self.callback18, queue_size=1)
    self.sub_upper_g_v = rospy.Subscriber("entrance_gate/upper_threshold_g/value", Int16, self.callback19, queue_size=1)


  def callback2(self, data):
	global lower_threshold
	lower_threshold["red"][0] = data.data

  def callback3(self, data):
  	global lower_threshold
	lower_threshold["red"][1] = data.data

  def callback4(self, data):
  	global lower_threshold
	lower_threshold["red"][2] = data.data

  def callback5(self, data):
  	global upper_threshold
	upper_threshold["red"][0] = data.data

  def callback6(self, data):
  	global upper_threshold
	upper_threshold["red"][1] = data.data

  def callback7(self, data):
  	global upper_threshold
	upper_threshold["red"][2] = data.data 


  def callback14(self, data):
	global lower_threshold
	lower_threshold["green"][0] = data.data

  def callback15(self, data):
  	global lower_threshold
	lower_threshold["green"][1] = data.data

  def callback16(self, data):
  	global lower_threshold
	lower_threshold["green"][2] = data.data

  def callback17(self, data):
  	global upper_threshold
	upper_threshold["green"][0] = data.data

  def callback18(self, data):
  	global upper_threshold
	upper_threshold["green"][1] = data.data

  def callback19(self, data):
  	global upper_threshold
	upper_threshold["green"][2] = data.data 

  def callback(self,data):
  	global image_type
  	global toss_images
  	global lower_threshold
  	if toss_images <= toss_images_threshold:
  		toss_images += 1
	else:
		toss_images = 0

		try:

		  if image_type == "Image": 
			  cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		  else:	
			  cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
		  # cropped_image = cv_image[100:250, 100:550]
		  y = cv_image.shape[0]
		  x = cv_image.shape[1]
		  x_min = int(x * 0.25)
		  x_max = int(x * 0.75)
		  y_min = int(y * 0.35)
		  y_max = int(y * 0.75)

		  cropped_image = cv_image[y_min:y_max, x_min:x_max]
		  cropped_image = cv2.blur(cropped_image,(7,7))
		  plot_mask(cropped_image, "Cropped Image", 3)
		  cv_image_hsv = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)
		except CvBridgeError as e:
		  print(e)

	  	# print(data.header.stamp.secs)

		classified, coordinates, radius_dict, mask_dict, res_dict = color_recognition(cv_image_hsv)

		def entrance_gate_indicator(classified, coordinates, x_pixels):

			if classified["red"] and classified["green"]:

				# Check to see if red buoy to left of green buoy by a margin of 10% of image width
				if (int(coordinates["red"][0]) + x_pixels * 0.1) < int(coordinates["green"][0]):
					return "True"
			
			return "False"

		self.pub_entrance_gate.publish(entrance_gate_indicator(classified, coordinates, x))

		for color in coordinates.keys():
			if classified[color]:
				x = int(coordinates[color][0])
				y = int(coordinates[color][1])
				r = int(radius_dict[color])

				if color == 'red':
					cv2.circle(cv_image, (x+x_min, y+y_min), r, (0,0,255))
				elif color == 'blue':
					cv2.circle(cv_image, (x+x_min, y+y_min), r, (255))
				elif color == 'green':
					cv2.circle(cv_image, (x+x_min, y+y_min), r, (0,255,0))
				elif color == "yellow":
					cv2.circle(cv_image, (x+x_min, y+y_min), r, (0,255,255))
				elif color == "gray":
					cv2.circle(cv_image, (x+x_min, y+y_min), r, (95,95,95))
				else:
					cv2.circle(cv_image, (x+x_min, y+y_min), r, (0,0,0))

		cv2.namedWindow('Image window',cv2.WINDOW_NORMAL)
		cv2.moveWindow('Image window', 40,30)
		cv2.resizeWindow('Image window', 1100,700)
		cv2.imshow("Image window", cv_image)
		cv2.waitKey(1)


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