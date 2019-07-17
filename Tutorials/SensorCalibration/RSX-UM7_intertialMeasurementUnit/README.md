# Redshift Labs RSX-UM7 Inertial Measurement Unit

## Quick start

To get started with the RSX-UM7 IMU, [read through this quick-start guide here](https://www.redshiftlabs.com.au/um7-quick-start-guide)

## Calibration

To calibrate the RSX-UM7 IMU, [read through this calibration procedure here](https://www.redshiftlabs.com.au/um7-calibration-procedure)

## ROS Node

To install the ROS `um7` package, [see the package wiki here](http://wiki.ros.org/um7)

[The Github repository can be found here](https://github.com/ros-drivers/um7)

If you need a refresher on how to install ROS packages, [see our tutorial here](https://github.com/riplaboratory/Kanaloa/blob/master/Tutorials/SoftwareInstallation/ROS/InstallingPackages/README.md)

## Coordinate Systems (THIS IS SUPER IMPORTANT)

The RSX-UM7 sensor internally employs a x-forward, y-right, z-down coordinate system.  This is the coordinate system printed on top of the sensor.  It looks something like this: 

![UM7 coordinate system](Images/IMG_20190716_193401.jpg)

Using the Redshift Serial Interface software, we have confirmed that this coordinate system is consistent for the accelerometer, rate gyroscope, and fused output.  

Unfortunately, at the time of writing (2019/07/16), the [`um7` ROS package](http://wiki.ros.org/um7) adopts a different coordinate system, and does so in an inconsistent manner.  
