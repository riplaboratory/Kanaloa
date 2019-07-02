# ROS Melodic Installation

## Prerequisites

 1. Ubuntu 18.04 LTS (http://releases.ubuntu.com/18.04/)

## Installation 

 1. To install ROS Melodic, follow these instructions: http://wiki.ros.org/melodic/Installation/Ubuntu
 
 2. Next, you must install `catkin_tools`.  Standard ROS tutorials have you install `CMake` as your catkin build system (documented here: http://wiki.ros.org/catkin#Installing_catkin); however, we are going to diverge from this advice, and use `catkin_tools`, which is generally accepted as better.  To do this, in terminal, type:
 
 ```sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
 wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
 ```
 
 These instructions came from this tutorial: https://catkin-tools.readthedocs.io/en/latest/installing.html
 
 
