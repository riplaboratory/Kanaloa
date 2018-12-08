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

# lower_red = np.array([140,30,30])
# upper_red = np.array([179,255,255])

# lower_blue = np.array([108,60,60])
# upper_blue = np.array([113,120,120])

# lower_yellow = np.array([12,30,30])
# upper_yellow = np.array([26,255,255])

# x_min = 100
# x_max = 550
# y_min = 100
# y_max = 250

x_min = 400
x_max = 1000
y_min = 200
y_max = 450

min_radius = 10

mask_plotting = True

# lower_red = np.array([140,120,60])
# upper_red = np.array([176,255,200])

# lower_blue = np.array([107,170,60])
# upper_blue = np.array([115,220,225])

# lower_yellow = np.array([12,30,30])
# upper_yellow = np.array([26,255,255])

# lower_green = np.array([74,30,30])
# upper_green = np.array([85,255,255])

# lower_gray = np.array([108,60,60])
# upper_gray = np.array([113,120,120])

# lower_threshold = {"red": lower_red,"blue": lower_blue, "yellow": lower_yellow, "green": lower_green, "gray": lower_gray}
# upper_threshold = {"red": upper_red,"blue": upper_blue, "yellow": upper_yellow, "green": upper_green, "gray": upper_gray}

# color_codes = {"red": (0,0,255), "blue": (255,0,0), "yellow": (0,255,255), "green": (0, 255, 0), "gray": (192,192,192)}

lower_red = np.array([0,90,120])
upper_red = np.array([5,230,200])

lower_blue = np.array([100,60,60])
upper_blue = np.array([112,255,190])

lower_green = np.array([40,50,50])
upper_green = np.array([72,210,210])

lower_threshold = {"red": lower_red,"blue": lower_blue, "green": lower_green}
upper_threshold = {"red": upper_red,"blue": upper_blue, "green": upper_green}

color_codes = {"red": (0,0,255), "blue": (255,0,0), "green": (0, 255, 0)}

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
		if topic_type[1] == "CompressedImage" or topic_type == "	Image":
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
    
    # lower_threshold = {"red": lower_red,"blue": lower_blue, "yellow": lower_yellow}
    # upper_threshold = {"red": upper_red,"blue": upper_blue, "yellow": upper_yellow}
    global lower_threshold
    global upper_threshold
    
    for color in lower_threshold.keys():
        lower_color = lower_threshold[color]
        upper_color = upper_threshold[color]
        # print(type(image))
        # print(image.shape)
        # image = image[100:200, 50:600]
        mask = cv2.inRange(image, lower_color, upper_color)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]
        res = cv2.bitwise_and(image, image, mask = mask)
        res_plot = cv2.cvtColor(res, cv2.COLOR_HSV2BGR)

        if mask_plotting:
        	if color == "red":
        		plot_mask(res_plot, color, 0)
    		if color == "blue":
        		plot_mask(res_plot, color, 1)
    		if color == "green":
        		plot_mask(res_plot, color, 2)
    		# if color == "yellow":
      #   		plot_mask(res_plot, color, 3)

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
    # self.image_pub = rospy.Publisher("image_topic_2",Image)
    print("Initializing Color Recognition \nA new window will appear once image are detected ...")
    self.bridge = CvBridge()
    # self.image_sub = rospy.Subscriber("camera1/usb_cam/image_raw/compressed",CompressedImage,self.callback, queue_size=1)
    if image_type == "Image":
	    self.image_sub = rospy.Subscriber(image_topic,Image,self.callback, queue_size=1)
    else:
	    self.image_sub = rospy.Subscriber(image_topic,CompressedImage,self.callback, queue_size=1)
	    # self.test_sub = rospy.Subscriber("chatter", String, self.callback2, queue_size=1)
    self.sub_lower_r_h = rospy.Subscriber("lower_threshold_r/hue", Int16, self.callback2, queue_size=1)
    self.sub_lower_r_s = rospy.Subscriber("lower_threshold_r/saturation", Int16, self.callback3, queue_size=1)
    self.sub_lower_r_v = rospy.Subscriber("lower_threshold_r/value", Int16, self.callback4, queue_size=1)
    self.sub_upper_r_h = rospy.Subscriber("upper_threshold_r/hue", Int16, self.callback5, queue_size=1)
    self.sub_upper_r_s = rospy.Subscriber("upper_threshold_r/saturation", Int16, self.callback6, queue_size=1)
    self.sub_upper_r_v = rospy.Subscriber("upper_threshold_r/value", Int16, self.callback7, queue_size=1)
    self.sub_lower_b_h = rospy.Subscriber("lower_threshold_b/hue", Int16, self.callback8, queue_size=1)
    self.sub_lower_b_s = rospy.Subscriber("lower_threshold_b/saturation", Int16, self.callback9, queue_size=1)
    self.sub_lower_b_v = rospy.Subscriber("lower_threshold_b/value", Int16, self.callback10, queue_size=1)
    self.sub_upper_b_h = rospy.Subscriber("upper_threshold_b/hue", Int16, self.callback11, queue_size=1)
    self.sub_upper_b_s = rospy.Subscriber("upper_threshold_b/saturation", Int16, self.callback12, queue_size=1)
    self.sub_upper_b_v = rospy.Subscriber("upper_threshold_b/value", Int16, self.callback13, queue_size=1)
    self.sub_lower_g_h = rospy.Subscriber("lower_threshold_g/hue", Int16, self.callback14, queue_size=1)
    self.sub_lower_g_s = rospy.Subscriber("lower_threshold_g/saturation", Int16, self.callback15, queue_size=1)
    self.sub_lower_g_v = rospy.Subscriber("lower_threshold_g/value", Int16, self.callback16, queue_size=1)
    self.sub_upper_g_h = rospy.Subscriber("upper_threshold_g/hue", Int16, self.callback17, queue_size=1)
    self.sub_upper_g_s = rospy.Subscriber("upper_threshold_g/saturation", Int16, self.callback18, queue_size=1)
    self.sub_upper_g_v = rospy.Subscriber("upper_threshold_g/value", Int16, self.callback19, queue_size=1)


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

  def callback8(self, data):
	global lower_threshold
	lower_threshold["blue"][0] = data.data

  def callback9(self, data):
  	global lower_threshold
	lower_threshold["blue"][1] = data.data

  def callback10(self, data):
  	global lower_threshold
	lower_threshold["blue"][2] = data.data

  def callback11(self, data):
  	global upper_threshold
	upper_threshold["blue"][0] = data.data

  def callback12(self, data):
  	global upper_threshold
	upper_threshold["blue"][1] = data.data

  def callback13(self, data):
  	global upper_threshold
	upper_threshold["blue"][2] = data.data 

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

		def generate_scanned_code_list(classified):
			global classified_colors_list
			global times
			# if classified.values().count(True) == 1:
			if classified["red"] == True or classified["blue"] or classified["green"]:
				
				c  = classified.keys()[classified.values().index(True)]
				if len(classified_colors_list) == 0 and (c == "red" or c == "green" or c == "blue"):
					classified_colors_list.append(c)
					times.append(data.header.stamp.secs)
				
				if classified_colors_list[-1] != c and (c == "red" or c == "green" or c == "blue"):
					classified_colors_list.append(c)
					times.append(data.header.stamp.secs)
			if len(times) > 2:
				if times[-1] < times[-2]:
					times = []
					classified_colors_list = []
					print("ERROR: went back in time")
			# print(times)
			if len(classified_colors_list) == 7:
				# print "Identified Color list: ", classified_colors_list
				# print "times: ", times
				print(scan_the_code(classified_colors_list, times))
				c = classified_colors_list
				t = times
				classified_colors_list.pop(0)
				times.pop(0)

				if  c[0] == c[3] and c[1] == c[4] and c[2] == c[5]:
					scanned = scan_the_code(c, t)
					plot_scan_the_code(scanned)


		generate_scanned_code_list(classified)

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

		# cv_image = cv_image[100:200, 50:600]

		cv2.namedWindow('Image window',cv2.WINDOW_NORMAL)
		cv2.moveWindow('Image window', 40,30)
		cv2.resizeWindow('Image window', 1100,700)
		cv2.imshow("Image window", cv_image)
		cv2.waitKey(1)

	    # try:
	    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
	    # except CvBridgeError as e:
	    #   print(e)


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