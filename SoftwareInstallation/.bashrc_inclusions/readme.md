# .bashrc Inclusions
The .bashrc file is a shell script that runs every time a new terminal is launched.  This file autosets inclusions to each terminal window for convenience.  Listed below are all of our standard .bashrc file inclusions.

This file is located in your home directory.  It is a hidden file, so you will not see it in your file explorer by default.  You can access it by:
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
#export ROS_MASTER_URI=http://10.42.0.58:11311/ 	# uncomment for Raspberry Pi (you have to change the IP to the IP of the Raspberry Pi!)

# Anaconda3 inclusions
export PATH="/home/brennan/anaconda3/bin:$PATH"

# virtualenv and virtualenvwrapper inclusions
export WORKON_HOME=$HOME/.virtualenvs
source /usr/local/bin/virtualenvwrapper.sh
```
