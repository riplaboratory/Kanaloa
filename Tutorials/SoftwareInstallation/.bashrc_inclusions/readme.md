# .bashrc Inclusions
The .bashrc file is a shell script that runs every time a new terminal is launched.  This file autosets inclusions to each terminal window for convenience.  Listed below are all of our standard .bashrc file inclusions.

## Prerequisites
   1. Ubuntu 16.04 ([instructions here](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/Ubuntu)).

## Instructions
The .bashrc file is located in your home directory.  It is a hidden file, so you will not see it in your file explorer by default.  You can access it by:

```
cd ~
gedit .bashrc
```

Once open, add the following lines to the end of the file:

```
# Robot Operating System (ROS) inclusions
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_HOSTNAME=10.42.0.1
export ROS_IP=10.42.0.1
export ROS_HOSTNAME=localhost
export ROS_IP=localhost
export ROS_MASTER_URI=http://localhost:11311/		# Uncomment for local machine
#export ROS_MASTER_URI=http://10.42.0.58:11311/ 	# uncomment for Raspberry Pi

# added by Anaconda2 installer
export PATH="/home/brennan/anaconda2/bin:$PATH"
```

Any terminal sessions you open after saving will reflect these inclusions.  You can update your current terminal session with these changes without closing it `.bashrc` by typing:

```
source ~/.bashrc
```
