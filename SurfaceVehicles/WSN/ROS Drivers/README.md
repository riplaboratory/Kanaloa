These are the ROS drivers used by the current WSN system and the sensors they are needed for. These packages will need to be on on any machine you start running the sensor nodes on. As the sensors are currently configured to run on the WAM-V computer, all of these packages are already on the WAM-V computer. 

These packages would be placed in the `src` folder of a ROS catkin workspace. More on [catkin workspaces](http://wiki.ros.org/catkin/workspaces) and [how to create them](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

|ROS Driver| Sensor |
|--|--|
| [nmea_navsat_driver](https://github.com/ros-drivers/nmea_navsat_driver.git) | ZED F9P GPS |
| [ros_f9p_driver](https://github.com/MrCerealKiller/ros_f9p_driver.git) | ZED F9P GPS |
| [ros_imu_bno055](https://github.com/RoboticArts/ros_imu_bno055.git) |BNO055 ROS Driver |
| [usb_cam](https://github.com/ros-drivers/usb_cam.git) | Logitech USB Camera |
