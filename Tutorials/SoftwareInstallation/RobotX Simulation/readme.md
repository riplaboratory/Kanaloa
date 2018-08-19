# Installing RobotX Simulation

This tutorial assumes that you already have Ubuntu 16.04 and ROS Kinetic installed on your machine. Team Kanaloa takes no credit for this project and should only be used as a reference by it's members. Original Publication credit goes to Brian Bingham, Carlos Aguero, and the RobotX community. There repository can be found here:  https://bitbucket.org/osrf/vmrc

## Prerequisites

1. Ubuntu 16.04 [instructions here](https://github.com/riplaboratory/Kanaloa/blob/master/Tutorials/SoftwareInstallation/Ubuntu16.04/readme.md)
2. Standard .bashrc inclusions [instructions here](https://github.com/riplaboratory/Kanaloa/blob/master/Tutorials/SoftwareInstallation/.bashrc_inclusions/readme.md)
3. ROS Kinetic [instructions here](https://github.com/riplaboratory/Kanaloa/blob/master/Tutorials/SoftwareInstallation/ROS/Kinetic/readme.md)

## Installing The Simulation
This tutorial will walk you through the setup required to make a computer ready to run the VMRC simulations. In order to run VMRC your computer will need a discrete graphics card and will need to satisfy the minimum System Requirements. All updates and official tutorials can be found here: https://bitbucket.org/osrf/vmrc/wiki/tutorials/

These instructions contain information for building the VMRC environment in Gazebo.

### Install all software in your host system
Because the simulation uses some relatively new (as of summer 2018) features in ROS and Gazebo, it is highly recommended that you upgrade the packages installed on your system:
```console
$ sudo apt update
$ sudo apt full-upgrade
```
### Setup and install dependencies:
```console
    $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

    $ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

    $ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

    $ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

    $ sudo apt update
```
    
```console
$ sudo apt install cmake mercurial gazebo7 git libeigen3-dev libgazebo7-dev pkg-config python ros-kinetic-gazebo-plugins ros-kinetic-gazebo-ros ros-kinetic-hector-gazebo-plugins ros-kinetic-joy ros-kinetic-joy-teleop ros-kinetic-robot-localization ros-kinetic-ros-base ros-kinetic-teleop-tools ros-kinetic-teleop-twist-keyboard ros-kinetic-velodyne-simulator ros-kinetic-xacro ruby wget
```
    
#### Now build a workspace for VMRC. 
If you are familiar with ROS catkin workspaces, this is a similar concept. The steps to setup the workspace are:

```console
$ mkdir -p ~/vmrc_ws/src
$ cd ~/vmrc_ws/src
```
Clone the VMRC repository:
```console
$ hg clone https://bitbucket.org/osrf/vmrc
```
Build instructions
Source the ROS setup.bash file:
```console
$ source /opt/ros/kinetic/setup.bash
```
Build all the software:
```console 
$ cd ~/vmrc_ws
$ catkin_make
```
Test Run
```console 
$ cd ~/vmrc_ws
$ source devel/setup.bash
$ roslaunch robotx_gazebo sandisland.launch 
```
## Thruster Configuration
There are currently 3 supported Propulsion options for the WAMV. 'H', 'T', and 'X'. Each individual thruster has it's own unique ROS topic name and can be manipulated through ROS scripts.
![alt text](https://bitbucket-assetroot.s3.amazonaws.com/repository/BgXLzgM/2101300599-Propulsion%20Options.png?Signature=FDYopYvj97CpMN3hCIZX%2Figg%2F2E%3D&Expires=1534632783&AWSAccessKeyId=AKIAIQWXW6WLXMB5QZAQ)
The default thruster configuration is the 'H' configuration. To utilize a different configuration a new parameter must be put in the roslaunch command.
##### 'T' Configuration
```console 
roslaunch robotx_gazebo sandisland.launch thrust_config:=T 
```
##### 'X' Configuration
```console
roslaunch robotx_gazebo sandisland.launch thrust_config:=X 
```
## Sensors
Creating a URDF file
A URDF file is a format to describe a robot including joints, sensors, inertial properties, and more. The file is used by Gazebo, rviz, and many other ROS packages. Several example URDF files for representing a WAM-V are included in the VMRC packages.

Let's copy an example locally as a starting point:

```console 
$ cd /path_to_vmrc_ws/vmrc_ws/src/vmrc/wamv_gazebo/urdf
```
```console
$ cp wamv_gazebo_sensors.urdf.xacro my_wamv.urdf.xacro
```
This file contains something like this:

```console
cat my_wamv.urdf.xacro
```
```xacro
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro"
       name="WAM-V">
  <!-- Basic frame of WAM-V -->
  <xacro:include filename="$(find wamv_gazebo)/urdf/wamv_gazebo.urdf.xacro" />

  <!-- ADD SENSORS -->
  <!-- Add a front camera -->
  <xacro:wamv_camera name="front_camera"/>
  <!-- Add simulated GPS -->
  <xacro:wamv_gps name="gps_wamv"/>
  <!-- Add Simulated IMU -->
  <xacro:wamv_imu name="imu_wamv"/>
  <!-- Add P3D ground truth -->
  <xacro:wamv_p3d name="p3d_wamv"/>
</robot>
```

Let's look at the contents within the <robot> tag, which describes the robot. The first line includes wamv_gazebo.urdf.xacro. This adds the basic WAM-V mesh and joints along with the plugins for dynamics. You will likely want to keep this in, unless you are using a different model or dynamics simulation.

After that several macros are added for a GPS, IMU, and ground truth pose. These macros are found in wamv_gazebo for common sensors. You can of course create your own, following those as examples.

Let's add a stereo camera pair to the robot. Add the following lines after the other sensors:

  <xacro:property name="stereo_x" value="1.0" />
  <xacro:wamv_camera name="stereo_left" x="${stereo_x}" y="0.3" z="1.5" P="${radians(15)}" />
  <xacro:wamv_camera name="stereo_right" x="${stereo_x}" y="-0.3" z="1.5" P="${radians(15)}" />
A couple things to notice about this:

A common property "stereo_x" is used so the value is not copied in multiple places
The x,y,z and P (pitch) set where the cameras are located relative to the WAM-V base link
A python expression ${radians(15)} was used to convert 15 degrees to radians
Running robotx_gazebo with a custom WAM-V URDF
Now that you have a custom URDF modeling your WAM-V, let's run the simulation!

First, generate the compiled XML from the xacro file using this or another method:

```console 
$ rosrun xacro xacro --inorder my_wamv.urdf.xacro > my_wamv.urdf
```
Next, run the simulation with a custom urdf argument:

```console 
$ roslaunch robotx_gazebo sandisland.launch urdf:=`pwd`/my_wamv.urdf
```
You can use rqt to see your sensor topics
## Holonomic Drive and Sensors
By default when using sensors, the "H" configuration is used in the simulation. So running:
```console 
$ roslaunch robotx_gazebo sandisland.launch urdf:=`pwd`/my_wamv.urdf  thrust_config:=X
```
Will run the simulation with the sensors, but the thrust configuration will remain as the default "H" Layout. To change this you must edit the file wamv_gazebo.urdf.xacro. There are two ways of doing this. 
##### 1) The "Easy" Way
You may do it the "easy way" and replace your wamv_gazebo.urdf file with the one posted in the Kanaloa Repository [here](https://github.com/riplaboratory/Kanaloa/blob/master/Tutorials/SoftwareInstallation/RobotX%20Simulation/vmrc_ws/src/vmrc/wamv_gazebo/urdf/my_wamv.urdf). Please note that /path_to_vmrc_ws is the location of where you downloaded your simulation on your machine.
 - Remove the wamv_gazebo.urdf file 
```console
$ cd /path_to_vmrc_ws/vmrc_ws/src/vmrc/wamv_gazebo/urdf/
```
```console 
$ rm wamv_gazebo.urdf.xacro
```
(or just use the Ubuntu File Manager application and visually delete the file)
 - [Click this link](https://raw.githubusercontent.com/riplaboratory/Kanaloa/master/Tutorials/SoftwareInstallation/RobotX%20Simulation/vmrc_ws/src/vmrc/wamv_gazebo/urdf/my_wamv.urdf)
 - Right click on the page and select "Save As"
 - Put in the name: wamv_gazebo.urdf
 - For the file type, make sure "All Files" is selected
 - Save to: /path_to_vmrc_ws/vmrc_ws/src/vmrc/wamv_gazebo/urdf/

##### 2) The "I'm a hardcore ROS developer" way
This method involved editing the wamv_gazebo.urdf.xacro file. Please note that in your simulation files there are two similarly named files, wamv_gazebo.urdf.xacro, and wamv_gazebo.urdf. Both files are located in: /path_to_vmrc_ws/vmrc_ws/src/vmrc/wamv_gazebo/urdf/

 - Open your favorite text editor
 - Edit line 7 so that
```xacro 
<xacro:wamv_gazebo thruster_layout="$(find wamv_gazebo)/urdf/thruster_layouts/wamv_aft_thrusters.xacro"/>
```
 - instead reads:
```xacro 
<xacro:wamv_gazebo thruster_layout="$(find wamv_gazebo)/urdf/thruster_layouts/wamv_x_thrusters.xacro"/>
```
 - Convert the XACRO file into a URDF
```console
$ rosrun xacro xacro --inorder wamv_gazebo.urdf.xacro > wamv_gazebo.urdf
```
 - Run the simulation
```console 
$ roslaunch robotx_gazebo sandisland.launch urdf:=`pwd`/my_wamv.urdf  thrust_config:=X
```

** Remeber to source your simulation every time you open a new terminal:
```console
$ source /path_to_vmrc_ws/vmrc_ws/devel/setup.bash
```

