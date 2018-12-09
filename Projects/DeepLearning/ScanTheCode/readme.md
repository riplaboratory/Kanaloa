# ROS Scan the Code Color Recognition

This script takes live image data, or image data from a bag file, that is published to a ROS topic and applys color recognition to the photo. This is mainly used for the RobotX Scan The Code task, in which a series of 3 colors will flash, and then the robot must identify that pattern. This script uses the OpenCV library on Python 2. The apporach to creating this code can be found in the 'Official Project Notebook.ipynb'

![alt text](https://github.com/riplaboratory/Kanaloa/blob/master/Projects/DeepLearning/ScanTheCode/images/overview.png)

## Prerequisites
1. Ubuntu 16.04 [instructions here](https://github.com/riplaboratory/Kanaloa/blob/master/Tutorials/SoftwareInstallation/Ubuntu16.04/readme.md)
2. ROS Kinetic [instructions here](https://github.com/riplaboratory/Kanaloa/blob/master/Tutorials/SoftwareInstallation/ROS/Kinetic/readme.md)

## Running the Script
To start the script and begin color recognition, first make sure roscore is running:
```
roscore
```
and then you can run the script using 
```
python2 color_recognition.py
```
When you run the scipt, you will be introduced to a series of prompts. These prompts help adjust the code to the image type of your image and to know where it is being published. The first prompt asks is
```
Name of the topic which the Images are Published (ex: topic_name/sub_topic): 
```
Do not start the topic name with a '/'. If you are unsure of the topic name that is being published, open another terminal and use the command
```
rostopic list
```
After you enter in the topic name, you will be prompted
```
Will you be subscribing to images of message type Image (i) or CompressedImage (c)  (i/c):
```
If you do not know wheather your image is a compressed image or normal image use the command
```
rostopic type /topic_name
```
replacing /topic_name with the actual name of the image topic you are trying to identify the type of.<br>
You will then be prompted to ensure you entered in the right inputs, for example
```
Images will be subscribed from topic  test 
with message type:  Image  

... Is this correct? (y/n)
```
Entering 'y' will begin the color recognition script. Once the script detects images being published to the specified topic it will open a seperate window showing the image feed and begin circling the specific colors it identifies. 

## Adjusting HSV Thresholds for REd Blue Green color recognition:
The GUI to adjust the threshold values of the color recognition can be run through the following command in a seperate window:
```
python2 rgb_threshold_adjuster.py
```
When run, three windows with sliders will appear, one for Red, Blue, and Green. The top 3 sliders denote the lower range HSV value amd the lower 3 sliders denote the upper range HSV values. These values can be changed using the sliders and then hitting the "Show" button to submit the changes to the scan_the_code.py script. Some default value use for the compeition are as follows:
```
lower_red = np.array([0,90,120])
upper_red = np.array([5,230,200])

lower_blue = np.array([100,60,60])
upper_blue = np.array([112,255,190])

lower_green = np.array([40,50,50])
upper_green = np.array([72,210,210])
``` 
Where the three values represent Hue, Saturation, and Value respectively. The value for Hue is run on a scale of 0-180 and the Saturation and Value values are on a scale of 0-255.

To exit the rgb_threshold_adjuster.py script, go to the terminal it is running in and  hit "Ctrl + C", this will kill the backend of the script. However you must then close all three slider windows to finish closing the graphical portion of it. If this is not done the slider GUI may remain stuck on your screen and can only be removed by restarting the computer or using the xkill command in terminal. 

### Using Images From A Bag File 
Before starting to script you must have ROS core running in a terminal with the command
```
roscore
```
Open another terminal (or new tab with Ctrl + Shift + T) and run thorugh the bag file you are trying to recieve images from
```
rosbag play ~/path_to/bag_file.bag
```
As the bag file is running you can open another teminal (or new tab with Ctrl + Shift + T) and ensure that the topic you are interested in is being published using the command:
```
rostopic list
```
And check the type of images that are coming in using the command
```
rostopic type /topic_name
```
Once you have all the information you need for the on screen prompts, you can stop running the bag file using Ctrl + C. Run the script and go through the dialogue with 
```
python2 color_recognition.py
```
Now you may run the bag file again using:
```
rosbag play ~/path_to/bag_file.bag
```
After finishing the on screen prompts, a window with the image feed from the topic should appear. If it does not, ensure you ROS image topic is still being published to and you put in the information correctly.
___

## Scan The Code Results
Once the script as "confidently" determined what the code is, a seperate image window will appear with the prediction for what the color code is.
![alt text](https://github.com/riplaboratory/Kanaloa/blob/master/Projects/DeepLearning/ScanTheCode/images/scanned.png)

## Stopping the script
To stop the script just hit 'Ctrl + C' while having the terminal running the script selected. If this does not work or the script does not properly close 'Ctrl + Z' can be used. If the image windows are not responding just close them and select 'Force Quit'.