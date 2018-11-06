# ROS topic Image Collection and Sorting

This script is used to help sort images that come in via ROS topics. The script takes in images being published and save it to folders corresponding to common objects found in the 2018 RobotX competition based on keyboard input from the user. The script is currently setup to for the RobotX 2018 competition objects, such the image data can be saved in following folders:
/triangle /cross /circle /buoy /multiple /all_images /null
This can be utilized for live images coming through a topic or images contained in a bag file. This script can be used with images published with the message types Image or CompressedImage. when running the script, a image feed window will automatically appear once it detects images being published to the topic selected. 

![alt text](https://github.com/riplaboratory/Kanaloa/blob/master/Projects/DeepLearning/data_collection/overview.png)

## Prerequisites
1. Ubuntu 16.04 [instructions here](https://github.com/riplaboratory/Kanaloa/blob/master/Tutorials/SoftwareInstallation/Ubuntu16.04/readme.md)
2. ROS Kinetic [instructions here](https://github.com/riplaboratory/Kanaloa/blob/master/Tutorials/SoftwareInstallation/ROS/Kinetic/readme.md)

## Running the Script
Before running the script, ensure that you run the script in a folder which you want all your sorted image data to be saved.
Once in the folder, the script sort_images.py is a python 2.7 script and can be run in terminal. Since the script uses ROS topics, ROS core must be running before executing the script. 
Use the following commands:
```
roscore
```
Then open another terminal (or use Ctrl+T) and run the script:
```
python2 sort_images.py
```
This will run the script in ther teminal and a series of prompts will begin to appear.
___

### On Screen Prompts
After executing the script there will be a series of onscreen prompts to ensure the script is properly subscribing to image topics and saving them in properly. The first one will look something like this:
```
Ensure roscore is running. Run script with sudo if you are in a restricted directory


Ensure this script is run in the directory which you want to save the images to their respective folders

Folders will be created in directory: 
/home/user/RobotX/ros_scripts/data_collection  
if they do not exist


Name of the topic which the Images are Published (ex: topic_name/sub_topic): 
```
This prompts where the script is being executed and where the image files will be saved. You will then enter the name of your topic. DO NOT START YOUR TOPIC NAME WITH A '/'. When rospy looks for a topic it already includes the beginning '/' by default, thus a topic known as '/camera/image_raw' will instead be written as 'camera/image_raw'. Once you finish typing in the topic name hit the 'Enter' key to get the following prompt:
```
Number to start naming images (If no image data collected yet just type '0'): 
```
This will decide what number to start at when saving your images. When the sort_images.py script saves images it names the image with a number followed by a .jpg extension. If you have no image data yet you can simply start at 0. If you do have existing image data and would like to add more without erasing the old set, you can choose a starting number (ex: 200) to save the images starting with that image. After you enter your integer number you can hit the 'Enter' key to get to the prompt:
```
Will you be subscribing to images of message type Image (i) or CompressedImage (c)  (i/c):
```
Depending on whether you are using the ros message type Image or CompressedImage for your images changes the way the image needs to be saved. Here you will enter 'i' if using a normal Image message type or 'c' if using a CompressedImage message. After typing 'i' or 'c' and hitting the Enter key you will be prompted something like:
```
Images will be subscribed from topic  test 
with message type:  Image  
Starting at image number  20 

... Is this correct? (y/n)
```
This is just to ensure you entered your information correctly. Type 'y' to continue or 'n' to close the script. If everything is correct the script will begin saving images and prompt you with the following prompt:
```
Beginning Recording of image data, Listening to Publisher:  test 


All image data will be recorded in folder 'all images', to also save a photo in a particular object folder use the following key commands:

	 t: triangle 
	 c: cross 
	 o: circle 
	 b: buoy 
	 m: multiple
	 n: no objects present
	 ?: repeat this prompt

```
You may then type one of the following keys and Enter to start recording to a particular folder. For example if you see a triangle from your image feed, you may then type 't' and Enter which will give the folloing prompt:
```
t
Recording to folder:  triangle
```
This confirms that your images are now being saved to that particular folder. To stop saving to that folder simply type 'n' and Enter which will prompt:
```
n
Not Recording to Object Folders
```
This indicates that you are no longer saving to that object folder, however the images will still be saved in the folder named 'all_images'.
If you come across images which have multiple opjects in them type 'm' and Enter to begin saving in the 'multiple' folder, this helps keep your data consistent so when training your classifier it is less likely to misclassify the object. 
```
m
Recording to folder: multiple
```
___

### Using Images From a Live ROS Topic
Before starting to script you must have ROS core running in a terminal with the command
```
roscore
```
As your ROS topics are being published, you can open another teminal (or new tab with Ctrl + Shift + T) and ensure that the topic you are interested in is being published using the command:
```
rostopic list
```
And check the type of images that are coming in using the command
```
rostopic type /topic_name
```
Once you have all the information you need for the on screen prompts, you can start the script with
```
python2 sort_images.py
```
After finishing the on screen prompts, a window with the image feed from the topic should appear. If it does not, ensure you ROS image topic is still being published to and you put in the information correctly. You may be moved away from the terminal running the script when the image appears, make sure to click back on the terminal to put in your key commands to sort the images.
___

### Using Images From A Bag File 
Before starting to script you must have ROS core running in a terminal with the command
```
roscore
```
Open another terminal (or new tab with Ctrl + Shift + T) and run thorugh the bag file
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
python2 sort_images.py
```
Now you may run the bag file again using:
```
rosbag play ~/path_to/bag_file.bag
```
And then select the console with the sort_image.py script running to put in your key commands to correspond to the folder the images are being saved to. When you are done you can exit the script with Ctrl + C. A window with the image feed from the topic should appear. If it does not, ensure you ROS image topic is still being published to and you put in the information correctly. You may be moved away from the terminal running the script when the image appears, make sure to click back on the terminal to put in your key commands to sort the images.
___

## Troubleshooting
Here are some problems that can be run into when trying to run the script

#### Script Not Recieving Images
There are generally two reasons why the script will not recieve the images, this will become apparent if your image files are still empty even when running the script with valid ROS topic images being published
1. ROS Master needs to be restarted : Sometimes the roscore buffer or cache gets corrupted which will leads the topics not properly being published to. This can be addressed by closing your current roscore terminal and launching another one with:
```
roscore
``` 
2. Topic name given in wrong format: Generally topics are given starting with a '/', however when rospy checks for topics, it ommits this beginning '/'. So when the propmt asks for the topic name, for example if you put '/camera/image_raw' then it will not find the topic. The topic must be typed such that it does not start with a '/' so for example 'camera/image_raw'

#### Can't stop running script / program crashes
This is mainly caused due to the Threading operation done in this script (essentially the script is running like 2 seperate python scripts), so sometimes one instance will not fully quit. If this occurs use the Ctrl+Z keyboard shortcut which will quit python. The image window may still be not responding, just exit by hitting the 'X' symbol in the top left and select "Force Quit"
