# Wireless Webcam Set-Up
This is a tutorial explaining the set-up for wireless transmission of video over the ROS using a USB camera, Raspberry Pi 3 Model B+, and 2.4 GHz WiFi. This has been tested with a Logitech QuickCam Pro 9000 and a Logitech C920 Pro HD webcam, the latter of which was installed and used for final testing in May 2021.

**This tutorial assumes you have completed the instructions outlined in the [Raspberry Pi set-up documentation](https://github.com/riplaboratory/Kanaloa/tree/master/SurfaceVehicles/WSN/Raspberry%20Pi%203B%2B%20Set-up)**

## Prerequisites
* Raspberry Pi 3 Model B+
* Logitech C920 Pro HD webcam (although this tutorial is likely to work with any standard USB camera)
* WiFi router
* PC or Laptop running Ubuntu 18.04 and ROS Melodic installed (this machine will be referred to as Master)
* [Power supply for Raspberry Pi](https://www.amazon.com/CanaKit-Raspberry-Supply-Adapter-Listed/dp/B00MARDJZ4) 

**Note:** The Raspberry Pi and Master must be connected to the same router's network. Network configuration is outlined in the Raspberry Pi set-up documentation (linked above).

### Accessing the Pi from your Laptop / PC using SSH
We use Secure Shell (SSH) in order to control the Pi from the Master PC or Laptop, which is made possible because the Raspberry Pi and Master is on the same network.

Open a terminal window on Master to ssh into the pi. `ssh` takes two parameters: *[username]@[IP address]*. Here is an example:
```
ssh kanaloa@192.168.0.104
```
 **Tip:** If you forget the IP address of the Pi but it is connected to the same network as Master, run the following in a terminal window on the Master (note that the first three numbers will need to match the first three numbers of the Master's IP address, and is subject to change (for example, 
 ```
 sudo nmap -sP 192.168.0.0/24
 or
 sudo nmap -sP 10.10.10.0/24 ##for the WAM-V computer, as the WAM-V PC IP address is 10.10.10.12
 ```
This will show the IP address of the all the devices on the same network as Master. Be patient, this command may take a around 30 seconds to complete. The IP address of the Pi will correspond with the device that is named  ```Raspberry Pi Foundation```

Once the ```ssh``` command is run, type ```yes``` to the authentication prompt (this will show up only the first time you ssh in) and then enter the password to the Pi when prompted. 

Your command line will now change to reflect your Pi's username, meaning you have successfully ssh'd in. 

**For the rest of this tutorial, when saying to run commands on the Raspberry Pi,  it is meant to run commands in this SSH terminal.**

### Install the Necessary ROS Packages and Drivers
  * usb_cam
  * v4l-utils
  * ros-melodic-image-view
  * net-tools

Using the command line, navigate to the ``/src`` directory in your sourced catkin workspace and install all of these on both the PC and RPi3B+:
```
cd ~/<name of your workspace>/src
git clone https://github.com/bosch-ros-pkg/usb_cam.git
cd ..
catkin_make
source ~/<name of your workspace/devel/setup.bash
sudo apt-get install v4l-utils
sudo apt-get install ros-melodic-image-view
sudo apt-get install net-tools
```
### Defining the Master and Client Devices
Before running the camera node, it is necessary to let the pi know which device will act as Master (the PC), and which will act as the slave or client device (the Pi). To do this, it is necessary to know the IP address of both devices. There are many ways to do this, but one is to run the following command in a terminal window of the device you are interested in:
```
hostname -I
```
Once you know the IP address of both devices, open the .bashrc file on your Pi (you do not have to be in a specific directory to do this)

```vi ~/.bashrc``` 

Add the following lines to the bottom of your .bashrc file to declare the Master and Client devices. 
```
export ROS_MASTER_URI=http://<IP address of your Master PC>:11311
export ROS_IP=<IP address of your Raspberry Pi>
```
Save the file and exit. You will have to restart your terminal and ssh back into the Pi for these changes to take effect.

## Launching the Camera Node
Of course, make sure that the USB camera is plugged into the pi. Also make sure that you have sourced your workspaces on both the Master and the Pi. 

In a terminal on Master, start ROS:
```
roscore
```

Next, on the Raspberry Pi, launch the usb_cam package:
```
roslaunch usb_cam usb_cam-test.launch
```
You may receive some warnings such as:
```
[ WARN] [1586989377.670962369]: unknown control 'white_balance_temperature_auto'
[ WARN] [1586989377.686331508]: unknown control 'focus_auto'
```
This is fine. The camera may just not have the functionality for automatic white balance or auto focus, but this will not affect transmission.

Next, on Master, run the following in another terminal window:
```
rosrun image_view image_view image:=/usb_cam/image_raw
```
A window will now pop up showing the live feed from the USB camera. Congrats.

## Editing Camera Parameters
The WSN team found that optimal camera settings for minimum lag and maximum visibility with wireless transmission were a 30 FPS frame rate and 480p resolution (height being 480 pixels and width being 640 pixels).

These parameters along with other parameters can be changed in the ```usb_cam-test.launch``` file located here:
```
~/<name of your workspace>/src/usb_cam/launch/usb_cam-test.launch
```

**NOTE for WAM-V integration: When implemented with the WAM-V and groundstation, the `usb_cam-test.launch` launch file *on the Pi* is edited to omit the second node. In the Master (which is the WAM-V PC) the launch file (wsn_launch.launch) only the first node, "usb_cam", is omitted. These lauch files are both indicated in this directory.**

## Resource
This documentation was heavily adapted from user yev-d on GitHub, found [here](https://github.com/yev-d/tutos/blob/master/ROS/Tutorial-001/Remotely-connecting-to-webcam-using-Raspberry-Pi-3-and-ROS.md).
