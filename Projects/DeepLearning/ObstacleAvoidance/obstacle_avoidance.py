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



x_min = 100
x_max = 550
y_min = 100
y_max = 250

def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

def color_recognition_mask_blue(image):
    
    lower_blue = np.array([120.2, 80.025, 60.0])
    upper_blue = np.array([152.2, 140, 100.0])

    mask = cv2.inRange(image, lower_blue, upper_blue)
    
    return mask


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
        		plot_mask(res_plot, "red", 0)
    		if color == "blue":
        		plot_mask(res_plot, "blue", 1)
    		if color == "green":
        		plot_mask(res_plot, "green", 2)

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            if radius > 15: 

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

  def callback(self,data):
  	global image_type
	try:

	  if image_type == "Image": 
		  cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	  else:	
		  cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
	  # cropped_image = cv_image[100:250, 100:550]
	  cropped_image = cv_image[y_min:y_max, x_min:x_max]
	  cv_image_hsv = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)
	except CvBridgeError as e:
	  print(e)

  	# print(data.header.stamp.secs)

	# blue_mask = color_recognition_mask_blue(cv_image_hsv)

	# warped, mask = perspect_transform(cropped_image_plot, source, destination)
	threshed = color_recognition_mask_blue(cv_image_hsv)

	# Calculate pixel values in rover-centric coords and distance/angle to all pixels
	xpix, ypix = rover_coords(threshed)
	dist, angles = to_polar_coords(xpix, ypix)
	mean_dir = np.mean(angles)


	# if mean_dir != 
	mean_dir = np.mean(angles)+1.5708 #Rotate 90 degrees
	print(mean_dir)
	print(type(mean_dir))

	x1 = cv_image.shape[1]/2
	y1 = cv_image.shape[0]
	arrow_length = 200
	if mean_dir >= 0:
		x2 = x1 - arrow_length * np.sin(mean_dir)
		y2 = y1 - arrow_length * abs(np.cos(mean_dir))
	else:
		x2 = x1 + arrow_length * np.sin(mean_dir)
		y2 = y1 - arrow_length * abs(np.cos(mean_dir))
	print "(X2, Y2) = ", x2, y2
	print "Type of (X2, Y2) = ", type(x2), type(y2)
	pt1 = (int(x1), int(y1))
	pt2 = (int(x2), int(y2))
	cv2.arrowedLine(cv_image, pt1, pt2, (0,0,255), 5)
	


	cv2.namedWindow('Image window',cv2.WINDOW_NORMAL)
	cv2.moveWindow('Image window', 40,30)
	cv2.resizeWindow('Image window', 1100,700)
	cv2.imshow("Image window", cv_image)
	cv2.waitKey(1)

	cv2.namedWindow('Mask',cv2.WINDOW_NORMAL)
	cv2.moveWindow('Mask', 1150,30)
	cv2.resizeWindow('Mask', 575,350)
	cv2.imshow('Mask', threshed)
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