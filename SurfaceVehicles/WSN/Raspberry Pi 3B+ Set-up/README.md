This is a Raspberry Pi 3B+ tutorial for setting up the following, all of which are needed to operate the wireless camera in WSN: 

 - Installing the **Ubuntu MATE 18.04** image. 
 - **Configuring headless operation** (so that the Pi connects to a wireless network automatically upon boot-up)
 -  Setting up Secure Shell Protocol (**SSH**).
 - Installing **ROS Melodic**

#### This tutorial assumes some familairity with the Linux command line.

## Prerequisites

 - Raspberry Pi 3B+
 -  [Power supply for Raspberry Pi](https://www.amazon.com/CanaKit-Raspberry-Supply-Adapter-Listed/dp/B00MARDJZ4) 
 - 16 GB micro SD card and SD card adapter
 - Monitor to use as display (HDMI cable also needed)
 - USB keyboard and mouse
 - Another computer with an SD card port in order to burn the Ubuntu image. 
	 - Your machine can be running Windows, Linux, or macOS

### Software

 - [balenaEtcher](https://www.balena.io/etcher/) must be installed on your machine (that has an SD card)

### Downloading and Flashing Ubuntu MATE 18.04
First, the appropriate Ubuntu MATE image will have to be flashed to an SD card, which is a file that will allow the Rasbpberry Pi to run Ubuntu 18.04. 

1. Available Ubuntu 18.04 images for Raspberry Pi are available on the [Ubuntu MATE website](https://releases.ubuntu-mate.org/archived/bionic/armhf/), but we use the following file: 

	[ubuntu-mate-18.04.2-beta1-desktop-armhf+raspi-ext4.img.xz](https://releases.ubuntu-mate.org/archived/bionic/armhf/ubuntu-mate-18.04.2-beta1-desktop-armhf+raspi-ext4.img.xz "ubuntu-mate-18.04.2-beta1-desktop-armhf+raspi-ext4.img.xz") 

	which will automatically download the Ubuntu MATE image to 		your computer once you click it.

	 **The .xz file has also been uploaded to the Kanaloa GitHub here ADD A LINK HERE TO WHERE IT IS**

2. Extract the contents of the downloaded image.
3. Using the adapter, insert your microSD card into your computer.
4. Open the balenaEtcher application
5. You will be instructed to select your image, which you just extracted, and the drive you are flashing to, which you have just inserted.
6. Flash the image. It may take several minutes, this is normal. 
7. Once finished, you can remove your micro SD card from your computer and it's adapter, and insert it into the Raspberry Pi. When powered, the Pi will now run the Ubuntu MATE image. 

### Turning on Pi for the First Time

 1. Before plugging the Pi in to power, make sure that the keyboard, mouse, and display monitor are all plugged into the Pi. (If you attempt to connect a display to the Pi after it has already been turned on, you will not get any output on the screen.)
 2. Connect the Pi to power. Once you get output on the screen, you will be walked through the process to set the time/date, login credentials, and the WiFi network you would like to connect to.
 
 ### Configuring Headless Operation
 **("Headless" refers to when the Pi is operating without any connected monitor, keyboard, or mouse.)**
 
 For the WSN, it is essential that the Pi is configured to connect to a preset wireless network upon boot-up while headless. 
 
 1. Open a terminal window on the Pi (Ctrl+Alt+t) and enter the following in the command line ('vim' can replaced by your preferred text editor).
	 ```
	sudo vim /usr/share/lightdm/lightdm.conf.d/40-lightdm-gtk-greeter.conf
	```
	
	If this file does not exist, navigate within the 		`lightdm` directory in order to find the appropriately named "lightdm" file.
3. Once the file has been opened, add a new "autologin" line under `[SeatDefaults]`as shown below, replacing "username" with the username of the Pi.
	```
	[SeatDefaults]
	greeter-session=lightdm-gtk-greeter
	autologin-user=username  
	```
4. Save and exit the file. Upon reboot, your Pi will bypass the login screen and automatically connect to the highest priority wireless connection you have set previously, without need for a monitor.

 ### Setting up Secure Shell Protocol (SSH)
Since we are running all operations from groundstation in WSN, we need to be able to ssh into the Raspberry Pi and run the launchfile that will turn on the camera.

There is an issue with the way ssh is set-up in this particular Ubuntu MATE image, so the following must be done in order to use ssh.

 1. Purge the exisitng ssh client that came with Ubuntu MATE
 `sudo apt-get remove openssh-client openssh-server`
 2. Update any outdated packages
 `sudo apt-get update`
 3. Reinstall the ssh client
 `sudo apt-get install openssh-server`

The ssh client is now properly set-up. More information on operating ssh is provided in the Setting up Wireless Webcam documentation INSERT LINK HERE

 ### Installing ROS Melodic
 The last thing needed on the Pi is ROS Melodic. This can be done through a terminal window on the Pi, or you may do this by ssh-ing into the Pi from another machine.
[Installing ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)

## Resources
[Headless setup](https://askubuntu.com/questions/1202230/auto-login-enable-on-turn-on-system)
