# Hector Slam SOP

This SOP (**S**tandard **O**perating **P**rocedure) was created to give instructions on how to run the [hector_slam](http://wiki.ros.org/hector_slam/Tutorials/MappingUsingLoggedData)
node with our current setup of our [UST-20LX Hokuyo Lidar](https://www.roscomponents.com/en/lidar-laser-scanner/86-ust-20lx.html).




## Configuration Setup of the Hokuyo Lidar

First, we will need to configure our network settings to be able to communicate with the Hokuyo lidar. The particular 
lidar we have uses an ethernet connection to send data to our device. 

1. Open the **network connections** menu by clicking on the small ![alt text](https://www.freepik.com/index.php?goto=27&url_download=aHR0cDovL3d3dy5mbGF0aWNvbi5jb20vZnJlZS1pY29uL3dpZmktY29ubmVjdGlvbi1zaWduYWwtc3ltYm9sXzUzNTI0&opciondownload=318&id=aHR0cDovL3d3dy5mbGF0aWNvbi5jb20vZnJlZS1pY29uL3dpZmktY29ubmVjdGlvbi1zaWduYWwtc3ltYm9sXzUzNTI0&fileid=788682 "wi-fi icon") icon.
2. Click on **edit connections**
3. Click **add**
4. For **choose a connection type** select **ethernet**
5. (Optional) Name the connection to something easy to read (e.g. I named mine "Hokuyo Lidar"). 
6. Go to the **IPv4 Settings** tab
7. Click on the dropdown next to **Method** and select **Manual**
8. Under the **Addresses** section, click **Add** 
9. Fill in the following: 

   | Address    | Netmask | Gateway   |
   |:----------:|:-------:|:---------:|
   |192.168.0.15| 24      |192.168.0.1|
   
10. Open a terminal (CTRL + ALT + T) and input the following: 
   
   ```sudo gedit /etc/network/interfaces```
   
11. You should see the following here:
   
   ```  
        1|  auto lo  
        2|  iface lo inet loopback 
   ```
        
12. *Underneath the code*, input the following, save, and exit: 

   ``` auto eth0  
       allow-hotplug eth0  
       iface eth0 inet static  
            address 192.168.0.1  
            netmask 255.255.255.0 
   ```
            
13. The Hokuyo Lidar should now be configured. For reference in troubleshooting, refer to [this](https://blog.csdn.net/hajungong007/article/details/79210140).

**_NOTE:_ When establishing a connection with the Hokuyo lidar, you will need to turn off your wi-fi connection and only have
the Hokuyo ethernet connection established. I.e. you will not have internet connection. There is probably a way to change this so 
you can still connect to the internet, but I didn't look too far into it.**




## Getting Data from the Hokuyo Lidar Using Urg_Node

Now that the network connection is configured, we can start getting data from it. Follow these steps:

1. Install the *urg_node* package by opening a terminal and typing in:
   
   ```sudo apt-get install ros-kinetic-urg-node```
   
   **_NOTE:_ This particular package was used along with ROS Distro Kinetic. If you are using a different ROS Distro, I 
   don't know if it will work. You can try replacing Kinitec with your distro to see if there is a package for it. Otherwise, 
   maybe try using the [hokuyo_node](http://wiki.ros.org/hokuyo_node) package.**
   
   ```sudo apt-get install ros-kinetic-urg-c```
   ```sudo apt-get install ros-kinetic-laser-proc```

2. By default, the Hokuyo lidar has an IP of 192.168.0.10. Again, there is probably a way to change this, but I didn't look into it. Run the urg_node package by inputting the following:

   ```rosrun urg_node urg_node _ip_address:="192.168.0.10"```

   It should say that it is streaming data now.

3. Not completely necessary for hector_slam, but if you wanna double-check if the lidar is actually getting data, you can
see what it looks like in rviz.

   ```rosrun rviz rviz```
   
   + Under **Global Options**, next to **Fixed Frame**, _manually_ type in "laser"
   + Click **Add** at the bottom, find the **Laser Scan** topic, and add it in
   + Under **Laser Scan** (now within the **Display** menu), select **/laser** as the topic. 
   



## Running the Hector_Slam Node
1. Install [hector_slam](http://wiki.ros.org/hector_slam) by opening a terminal and inputting the following:
   ```sudo apt-get install ros-kinetic-hector-slam``` 
   
2. For this particular configuration, odometry is not being used (it may be implemented later though.) Therefore, transformations involving odom need to be disabled. To do so, the launch file for the **hector_mapping** node must be edited. Type the following into a terminal (ensure **roscore** is running):

   ```roscd hector_mapping/launch```
   
   ```gedit mapping_default.launch```

3. Find these under the the **Tf Use** section and change these to look like this:

```  20     <param name="use_tf_scan_transformation" value="false"/>
     21     <param name="use_tf_pose_start_estimate" value="false"/>
     22     <param name="pub_map_odom_transform" value="false"/>
```     
   Save the settings. 
   
4. Last thing to do is simply launch the **hector_slam** launch file:

``` roslaunch hector_slam_launch tutorial.launch ```

