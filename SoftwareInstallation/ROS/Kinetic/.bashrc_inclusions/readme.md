# .bashrc Inclusions
Copy and past the following text at the end of your ROS .bashrc file.  In a default install of ROS, your .bashrc file will be located in your home directory (it is a hidden file, so you will not see it in the file explorer by default).  You can access it with
```
cd
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
