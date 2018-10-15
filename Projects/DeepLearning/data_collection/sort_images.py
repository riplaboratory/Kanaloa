#!/usr/bin/env python
print(chr(27) + "[2J")
print("Ensure roscore is running. Run script with sudo if you are in a restricted directory\n")
import rospy
import os
import time

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

def image_callback(msg, args):
	global image_counter
	global save_location
	global delay
	global save_type
	# print("Recieved Image")

	if save_type == "Image":
		cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
	else:	
		cv2_img = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

	cv2.namedWindow('Image window',cv2.WINDOW_NORMAL)
	cv2.resizeWindow('Image window', 1200,800)
	cv2.imshow("Image window", cv2_img)
	cv2.waitKey(1)

	if delay == 18:

		try:
		    # Convert your ROS Image message to OpenCV2
			# print(save_type)

			if save_type == "Image":
				cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
			else:	
				cv2_img = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

		except CvBridgeError, e:
		    print(e)
		else:
			# print("Recieved Image")
			# Save your OpenCV2 image as a jpeg 
			all_images_path = "all_images/" + str(image_counter) + ".jpg"
			cv2.imwrite(all_images_path, cv2_img)

			image_counter += 1

			# print(save_location)
			if save_location != "null":
				object_images_path = save_location + "/" + str(image_counter) + ".jpg"
				cv2.imwrite(object_images_path, cv2_img)

		delay = 0

	
	delay += 1
	# print(delay) 


def intro():
	print("\nEnsure this script is run in the directory which you want to save the images to their respective folders")
	current_directory = os.getcwd()
	print "\nFolders will be created in directory: \n", current_directory, " \nif they do not exist"
	image_topic = raw_input("\n\nName of the topic which the Images are Published (ex: topic_name/sub_topic): \n")
	image_counter = raw_input("\n\nNumber to start naming images (If no image data collected yet just type '0'): ")
	image_counter = int(image_counter)
	save_type = raw_input("\n\nWill you be subscribing to images of message type Image (i) or CompressedImage (c)  (i/c):\n")
	if save_type == 'c': save_type = 'CompressedImage'
	else: save_type = 'Image'
	print(chr(27) + "[2J")
	print "Images will be subscribed from topic ", image_topic, "\nwith message type: ", save_type, " \nStarting at image number ", image_counter, "\n\n... Is this correct? (y/n)"
	confirmation = raw_input("")


	if confirmation != "y":
		print("Please Restart the script and try entering the information correctly")
		assert confirmation == "y", "image topic and number not properly specified"

	print(chr(27) + "[2J")
	object_directories = ["/triangle", "/circle", "/cross", "/buoy", "/null", "/all_images", "/not_circle", "/not_triangle", "/not_cross", "/not_buoy", "/multiple"]

	for directory in object_directories:
		if not os.path.exists(current_directory+directory):
			os.makedirs(current_directory+directory)
			print directory, "directory created"
			
	print "\n\nBeginning Recording of image data, Listening to Publisher: ", image_topic, "\n\n"
	print("All image data will be recorded in folder 'all images', to also save a photo in a particular object folder use the following key commands:\n")
	print("\t t: triangle \n\t c: cross \n\t o: circle \n\t b: buoy \n\t m:multiple \n\t n: no objects present \n\t ?: repeat this prompt")

	return image_topic, image_counter, save_type

def save_image():
	# rospy.init_node('image_listener')
	global image_topic
	global save_type
	image_counter = 0
	while not rospy.is_shutdown():
		# print("Func 1")
		if save_type == "Image":
			rospy.Subscriber(image_topic, Image, image_callback, ("null", image_counter), queue_size = 1)
			rospy.spin()
		if save_type == "CompressedImage":
			rospy.Subscriber(image_topic, CompressedImage, image_callback, ("null", image_counter), queue_size = 1)
			rospy.spin()		# print(save_location)

def check_keystroke():
	global save_location
	while not rospy.is_shutdown():
		folder_key = raw_input("\n")  # Python 2

		if folder_key == "n":
		    save_locatoion = "null"
		    print("Not Recording to Object Folders")
		elif folder_key == "t":
		    save_location = "triangle"
		    print "Recording to folder: ", save_location
		elif folder_key == "c":
		    save_location = "cross"
		    print "Recording to folder: ", save_location
		elif folder_key == "o":
		    save_location = "circle"
		    print "Recording to folder: ", save_location
		elif folder_key == "b":
		    save_location = "buoy"
		    print "Recording to folder: ", save_location
		elif folder_key == "m":
		    save_location = "multiple"
		    print "Recording to folder: ", save_location
		elif folder_key == "?":
		    save_location = "null"
		    print("Not Recording to Object Folders")
		    print("\t t: triangle \n\t c: cross \n\t o: circle \n\t b: buoy \n\t m: multiple \n\tn: no objects present \n\t ?: repeat this prompt")
		else:
			save_location = "null"
			print("Not Recording to Object Folders")


if __name__ == '__main__':
	global image_counter
	global image_topic
	global delay
	global save_type

	delay = 0
	image_topic, image_counter, save_type = intro()
	save_location = '/null'

	while not rospy.is_shutdown() or KeyboardInterrupt:
		rospy.init_node('image_listener')
		Thread(target = save_image).start()
		Thread(target = check_keystroke).start()
		rospy.spin()