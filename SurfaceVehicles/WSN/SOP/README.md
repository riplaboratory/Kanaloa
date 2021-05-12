# Operating WSN on the WAM-V

This is the general SOP to get WSN (for the GPS, IMU, and Camera) running from Groundstation on the WAM-V computer, as it is currently (May 2021) set up. This tutorial assumes some familairity with ROS commands.

---
## Prerequisites

 - A laptop with Ubuntu 18.04 and ROS Melodic
 - Access to the WAM-V computer

In order for the laptop itself to view data (especially camera feed) you must first change the bashrc file so that it recognizes the WAM-V as the ROS Master. Open the file:
```
vi ~/.bashrc
```

At the end of the bashrc file (right before where the ROS environment was sourced works) add this line: 
	
```
export ROS_MASTER_URI=http://10.10.10.12:11311
```
	
Save and exit the file. Close and reopen your terminal so that the changes can take effect. 

**When you are finished working with WSN, make sure to comment out/delete this line from the bashrc file, as it will prevent roscore from working on the laptop.**

The F20-S21 Design Team has left the sensor nodes so they can be launched with (and only with) the WAM-V computer without having to upload or change anything in their code (there is extensive documentation about creating the code and configuring it for a different computer in the WSN directory).  A laptop is used to interface with the Raspberry Pi and the WAM-V computer.

---
**When encountering a password prompt in any part of the procedure, the password is `Aut0m@tion`**

## Procedure
 
 1. Turn on the WAM-V computer and router.
 2. Power on the sensor node boxes using the switches on their box covers. A red LED on the ESP32s and the Raspi will turn on when it is powered.
 3. If the WAM-V is getting deployed, make sure the Groundstation laptop is plugged into the ground station antenna via ethernet. Your laptop should show a wired connection has been made. If you will stay close to the WAM-V computer (the WAM-V is not getting deployed), connecting to the MRUH_2G WiFi network will suffice.
 4. On the laptop, open up a terminal window and ssh into the Raspberry Pi.  Make sure to give about 2 minutes for the Pi to boot up properly before trying to ssh. 
  
    ```
	  ssh kanaloa 10.10.10.136
    ```
 5. Once you are in, run the following command to turn on the camera node:
 
    ```
	  roslaunch usb_cam usb_cam-test.launch
    ```
  
  It is normal to see yellow or red warning messages.
  
7. Open a new terminal window and ssh into the WAM-V computer:
 
    ```
	  ssh kanaloa@10.10.10.12
    ```
 
 6. Once you are in, navigate to the WSN launch file and launch it to start the sensor nodes
	  
    ```
	  cd wsn
	  roslaunch wsn_launch.launch
	  ```
    
8. It may take anywhere from 5-20 seconds for transmission to start, but `rostopic list` and `rostopic echo` can be used to ensure there is data being published (see step 9 for viewing camera feed). If you have access to the node boxes, there is alos a blue LED on the ESP32s that indicate that the node has been started. Below are the names of the topics WSN team has assigned the sensors:

    |Sensor|ROS topic name  |
    |--|--|
    |IMU| imu_data/ |
    |GPS| fix/ |
    |Camera| usb_cam/image_raw/ |

9.  IMU and GPS data can be seen with `rostopic echo`, but the procedure is different for the camera. Open a third terminal window (no ssh required), and run the following:

    ```
	  rosrun image_view image_view image:=/usb_cam/image_raw
    ```

	A window should automatically open showing the camera feed. 
