# Installing RobotX Simulation

This tutorial assumes that you already have Ubuntu 16.04 and ROS Kinetic installed on your machine.

## Prerequisites

1. Ubuntu 16.04 [instructions here](https://www.google.com)
2. Standard .bashrc inclusions [instructions here](https://www.google.com)
3. ROS Kinetic [instructions here](https://www.google.com)

## Installing The Simulation
This tutorial will walk you through the setup required to make a computer ready to run the VMRC simulations. In order to run VMRC your computer will need a discrete graphics card and will need to satisfy the minimum System Requirements. All updates and official tutorials can be found here: https://bitbucket.org/osrf/vmrc/wiki/tutorials/

These instructions contain information for building the VMRC environment in Gazebo.

### Install all software in your host system
Because the simulation uses some relatively new (as of summer 2018) features in ROS and Gazebo, it is highly recommended that you upgrade the packages installed on your system:
   ```$ sudo apt update```
    ```$ sudo apt full-upgrade```
### Setup and install dependencies:
```$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'```
    ```$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116```
    ```$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'```
    ```$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -```
    ```$ sudo apt update```
    
```$ sudo apt install cmake mercurial gazebo7 git libeigen3-dev libgazebo7-dev pkg-config python ros-kinetic-gazebo-plugins ros-kinetic-gazebo-ros ros-kinetic-hector-gazebo-plugins ros-kinetic-joy ros-kinetic-joy-teleop ros-kinetic-robot-localization ros-kinetic-ros-base ros-kinetic-teleop-tools ros-kinetic-teleop-twist-keyboard ros-kinetic-velodyne-simulator ros-kinetic-xacro ruby wget```
    
#### Now build a workspace for VMRC. 
If you are familiar with ROS catkin workspaces, this is a similar concept. The steps to setup the workspace are:

```$ mkdir -p ~/vmrc_ws/src```
```$ cd ~/vmrc_ws/src```
Clone the VMRC repository:
    ```$ hg clone https://bitbucket.org/osrf/vmrc```
Build instructions
Source the ROS setup.bash file:
    ```$ source /opt/ros/kinetic/setup.bash```
Build all the software:
    ```$ cd ~/vmrc_ws```
    ```$ catkin_make```
Test Run
    ```$ cd ~/vmrc_ws```
    ```$ source devel/setup.bash```
    ```$ roslaunch robotx_gazebo sandisland.launch ```
## Thruster Configuration
Being worked on
## Sensors
Being Worked On
## Holonomic Drive and Sensors
Also Being worked on