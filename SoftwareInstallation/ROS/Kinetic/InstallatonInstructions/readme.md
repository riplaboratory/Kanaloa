# Installing ROS Kinetic
Last update: 2018.05.22

This tutorial assumes that you already have Ubuntu 16.04 installed on your machine.

## Prerequisites
   1. Ubuntu 16.04 ([instructions here](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/Ubuntu)).
   2. Standard .bashrc inclusions ([instructions here](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/.bashrc_inclusions)).

## Installation instructions
Detailed instructions can be found at the [ROS Wiki](http://wiki.ros.org/kinetic/Installation/Ubuntu):
   1. If you're not sure if you already have an existing version of ros, uninstall with:

```
sudo apt-get purge ros-*
```

   2. Setup your sources list and keys:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```

   3. Ensure debian package index is up-to-date:

```
sudo apt-get update
```

   4. Install

```
sudo apt-get install ros-kinetic-desktop-full
```
    Type `y`, then hit `enter` to confirm the install.

   5. Find available packages

```
apt-cache search ros-kinetic
```

   6. Initialize rosdep

```
sudo rosdep init
rosdep update
```

   7. Environment setup

```
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

   8. Install rosinstall

```
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
```
   Type `y`, then hit `enter` to confirm the install.

   9. Add our standard .bashrc inclusions.  Detailed instrcuctions can be found in [this directory](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/.bashrc_inclusions).

   10. Create ROS workspace

```
cd
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

## Install ROS Packages
Lists with installation instructions for groupings of ROS packages for various functionalities can be found [in our PackageLists directory](https://github.com/riplaboratory/Kanaloa/tree/master/ROS/Kinetic/PackageLists).  You may want to install all, or some of these package groups depending on the functionality you need from your ROS install.  
