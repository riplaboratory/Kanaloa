#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 00:59:04 2018

@author: brennan

Collects, and performs rough synchronization on images from two usb_cam devices.
for stereo vision.  roscore must be running, and the following topics must be
publishing:
    
    '/cameraLeft/usb_cam/image_raw/compressed'
    '/cameraRight/usb_cam/image_raw/compressed'
    '/cameraLeft/usb_cam/camera_info'
    '/cameraRight/usb_cam/camera_info'

Note that this script is not part of a ros package, and thus, can be run
directly.
        
Much help from:
    http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
    http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber
    https://answers.ros.org/question/220502/image-subscriber-lag-despite-queue-1/
    https://docs.opencv.org/3.1.0/da/df5/tutorial_py_sift_intro.html

"""
#%% Import calls

# Remove and append ROS Python 2.7 package directory from PYTHONPATH (changes hierarchy of import)
import sys
if any('/opt/ros/kinetic/lib/python2.7/dist-packages' in s for s in sys.path):
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
if any('/usr/lib/python2.7/dist-packages' in s for s in sys.path):
    sys.path.remove('/usr/lib/python2.7/dist-packages')
if not any('/opt/ros/kinetic/lib/python2.7/dist-packages' in s for s in sys.path):
    sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
if not any('/usr/lib/python2.7/dist-packages' in s for s in sys.path):
    sys.path.append('/usr/lib/python2.7/dist-packages')

# Imports
import time
import numpy as np
from scipy.ndimage import filters
import imageio
import rospy, roslib
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, CameraInfo
import cv2
import matplotlib.pyplot as plt

#%% Global variables

# User variables
verbose = True      # determines whether or not debug information is printed
nImg = 30           # number of image pairs to collect
imgPause = 1        # approximate time time to wait betweem image collection [s]

# Callback variables (preallocate memory)
left = np.zeros([1080,1920,3])      # left image
nsecL = 0                           # time left image callback was triggered
right = np.zeros([1080,1920,3])     # right image
nsecR = 0                           # time right image callback was triggered

#%% Classes

class imageCollect:
    
    def __init__(self):
        
        # Instantiate ROS imageCollect node
        rospy.init_node('imageCollect',anonymous=True)
               
        # Topic(s) node subscibes to
        self.leftSub = rospy.Subscriber('/cameraLeft/usb_cam/image_raw/compressed',CompressedImage,self.leftCallback,queue_size=1,buff_size=(2**24))
        self.rightSub = rospy.Subscriber('/cameraRight/usb_cam/image_raw/compressed',CompressedImage,self.rightCallback,queue_size=1,buff_size=(2**24))
        self.leftInfoSub = rospy.Subscriber('/cameraLeft/usb_cam/camera_info',CameraInfo,self.leftInfoCallback,queue_size=1,buff_size=(2**16))
        self.rightInfoSub = rospy.Subscriber('/cameraRight/usb_cam/camera_info',CameraInfo,self.rightInfoCallback,queue_size=1,buff_size=(2**16))
        
        # Local versions of callback variables (preallocate memory)
        self.left_ = np.zeros([1080,1920,3])
        self.right_ = np.zeros([1080,1920,3])
        
        if verbose:
            print '\nLaunching imageCollect ROS Python Node...'
            print '    subscribed to /cameraLeft/usb_cam/image_raw/compressed'
            print '    subscribed to /cameraRight/usb_cam/image_raw/compressed'
            
    def collectImages(self):
        
        if verbose:
            print '\nCollecting %s image pair(s)...' % nImg
        
        for i in range(1,nImg+1):
            
            # Synchonize incoming messages
            self.sync()
                        
            # Write to file
            self.write2file(i)  
            
            # Pause
            time.sleep(imgPause)
            
    def write2file(self,seq):
        
        # Start time epoch
        time0 = time.time()
        
        # Write to file
        cv2.imwrite(('left%s.jpg' %seq),self.left_,[cv2.IMWRITE_JPEG_QUALITY,100])
        time1 = time.time()
        cv2.imwrite(('right%s.jpg' %seq),self.right_,[cv2.IMWRITE_JPEG_QUALITY,100])
        time2 = time.time()
                
        if verbose:
            print '    left%s image written to file in %s sec' % (seq,(time1-time0))
            print '    right%s image written to file in %s sec' % (seq,(time2-time1))
    
    def sync(self):
        
        syncError = 0.01    # maximum allowable sync error [s]
        loops = 0
        
        # Loop until sync offset becomes smaller than syncError
        time0 = time.time()
        while abs(nsecL-nsecR) > syncError*1E9:
            loops = loops + 1
        time1 = time.time()

        # Save current callback image to local version           
        self.left_ = left
        self.right_ = right
        
        if verbose:
            print '    %s sec spent synchronizing images (%s loops)' % ((time1 - time0),loops)
            
        
    def leftCallback(self,rosData):
        
        # Convert to cv2
        npArr = np.frombuffer(rosData.data,np.uint8)    # convert compressed image to 1D numpy array
        npImg = cv2.imdecode(npArr,cv2.IMREAD_COLOR)    # convert numpy array to 3D numpy image array.  Note: cv2 uses BGR (instead of RGB).  
        
        # Save image to left global variable
        global left
        left = npImg[:,:,:]
        
    def rightCallback(self,rosData):

        # Convert to cv2
        npArr = np.frombuffer(rosData.data,np.uint8)    # convert compressed image to 1D numpy array
        npImg = cv2.imdecode(npArr,cv2.IMREAD_COLOR)    # convert numpy array to 3D numpy image array.  Note: cv2 uses BGR (instead of RGB).  
        
        # Save image to left global variable
        global right
        right = npImg[:,:,:]
        
    def leftInfoCallback(self,rosData):
        
        global nsecL
        nsecL = rosData.header.stamp.nsecs
                
    def rightInfoCallback(self,rosData):
        
        global nsecR
        nsecR = rosData.header.stamp.nsecs

#%% Main

if __name__ == '__main__':
    
    # Initialized
    print '\nInitializing...'
    print '    OpenCV Version: '+cv2.__version__
        
    # Instantiate Python imageCollect class
    ic = imageCollect()
    
    # Wait for one second (for node to initialize and publish)
    time.sleep(1)
    
    # Collect images
    ic.collectImages()
    
    # Keeps Python from exiting until node is stopped
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
