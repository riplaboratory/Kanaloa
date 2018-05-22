# Installing ROS Kinetic
Last update: 2018.05.22

## 1. Uninstall ROS
If you're not sure if you already have an existing version of ros, uninstall with:

```
sudo apt-get purge ros-*
```

## 2. Install ROS
Detailed instructions from the [ROS Wiki](http://wiki.ros.org/kinetic/Installation/Ubuntu):

1. Setup your sources list and keys:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```

2. Ensure debian package index is up-to-date:

```
sudo apt-get update
```

3. Install

```
sudo apt-get install ros-kinetic-desktop-full
```

4. Find available packages

```
apt-cache search ros-kinetic
```

5. Initialize rosdep

```
sudo rosdep init
rosdep update
```

6. Environment setup

```
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

7. Install rosinstall

```
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

8. Add our standard .bashrc inclusions.  Detailed instrcuctions can be found in [this directory](https://github.com/riplaboratory/Kanaloa/tree/master/ROS/Kinetic/.bashrc_inclusions)

9. Create ROS workspace

```
cd
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

## Install Packages
Lists with installation instructions for our standard ROS packages can be found [in our PackageLists directory](https://github.com/riplaboratory/Kanaloa/tree/master/ROS/Kinetic/PackageLists).  You may want to install all, or some of these packages depending on what you're doing with your ROS install.  
