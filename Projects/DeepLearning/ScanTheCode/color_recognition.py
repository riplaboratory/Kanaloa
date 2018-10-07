#!/usr/bin/env python
import rospy
import os
import time
import sys
import numpy as np

# from Tkinter import *

# master = Tk()
# w = Scale(master, from_=0, to=42)
# w.pack()
# w = Scale(master, from_=0, to=200, orient=HORIZONTAL)
# w.pack()
# print(w)

# mainloop()

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

lower_red = np.array([0,50,50])
upper_red = np.array([10,255,255])

lower_blue = np.array([115,50,50])
upper_blue = np.array([130,255,255])


lower_yellow = np.array([20,100,100])
upper_yellow = np.array([30,255,255])


def color_recognition(image):
    classified, coordinates, radius_dict, mask_dict, res_dict = {}, {}, {}, {}, {}
    
    lower_threshold = {"red": lower_red,"blue": lower_blue, "yellow": lower_yellow}
    upper_threshold = {"red": upper_red,"blue": upper_blue, "yellow": upper_yellow}
    
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

     #    red = {'mask': mask, 'res': res, 'coordinates': c }
     #    if color == "red":
     #    	mask_red = mask
     #    	res_red = res

    	# if color == "blue":
     #    	mask_blue = mask
     #    	res_blue = res

    	# if color == "yellow":
     #    	mask_yellow = mask
     #    	res_yellow = res


    return classified, coordinates, radius_dict, mask_dict, res_dict
    # return classified, coordinates, radius_dict

class image_converter:

  def __init__(self):
    # self.image_pub = rospy.Publisher("image_topic_2",Image)
    print("Initializing Class")
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("camera1/usb_cam/image_raw/compressed",CompressedImage,self.callback)

  def callback(self,data):
	try:
	  cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
	  cv_image_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
	  cv_image_hsv = cv_image_hsv[100:200, 50:600]
	except CvBridgeError as e:
	  print(e)

  	print(data.header.stamp.secs)

	# (rows,cols,channels) = cv_image.shape
	# if cols > 60 and rows > 60 :
	#   cv2.circle(cv_image, (425,75), 50, 255)

	classified, coordinates, radius_dict, mask_dict, res_dict = color_recognition(cv_image_hsv)

    # mask = np.zeros(cv_image.shape[])
	for color in coordinates.keys():
		if classified[color]:
			x = int(coordinates[color][0])
			y = int(coordinates[color][1])
			r = int(radius_dict[color])

			if color == 'red':
				cv2.circle(cv_image, (x, y), r, (0,0,255))

			elif color == 'blue':
				cv2.circle(cv_image, (x, y), r, (255))

			elif color == 'yellow':
				cv2.circle(cv_image, (x, y), r, (0,255,255))

			else:
				cv2.circle(cv_image, (x, y), r, (0,0,0))


    # mask = np.add(mask_dict["red"], mask_dict["blue"])
    # mask = np.add(mask, mask_dict["yellow"])

    # res = np.add(res_dict["red"], res_dict["blue"])
    # res = np.add(res, res_dict["yellow"])
    # res = cv2.cvtColor(res, cv2.COLOR_HSV2BGR)

	# cv_image = cv_image[100:200, 50:600]
	# print(type(cv_image))
	# print(cv_image.shape)
	cv2.namedWindow('Image window',cv2.WINDOW_NORMAL)
	cv2.resizeWindow('Image window', 1600,1000)
	cv2.imshow("Image window", cv_image)
	cv2.waitKey(3)

	cropped_image = cv_image[100:200, 50:600]
	cv2.namedWindow('Cropped Image window',cv2.WINDOW_NORMAL)
	cv2.resizeWindow('Cropped Image window', 1600,1000)
	cv2.imshow("Cropped Image window", cropped_image)
	cv2.waitKey(3)
    # cv2.imshow("Res window", res_blue)
    # cv2.waitKey(10)
    # cv2.imshow("Res window", res)
    # cv2.waitKey(3)
    # cv2.imshow("Red Mask window", mask_red)
    # cv2.waitKey(3)
    # cv2.imshow("Blue window", mask_blue)
    # cv2.waitKey(10)
    # cv2.imshow("Yellow window", mask_yellow)
    # cv2.waitKey(10)

    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)