# .bashrc Inclusions
Copy and past the following text at the end of your ROS .bashrc file.  In a default install of ROS, your .bashrc file will be located in your home directory (it is a hidden file, so you will see it in the file explorer by default).

```
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash

export ROS_HOSTNAME=10.42.0.1
export ROS_IP=10.42.0.1
export ROS_HOSTNAME=localhost
export ROS_IP=localhost

## Uncomment for local machine
export ROS_MASTER_URI=http://localhost:11311/

## Uncomment for pi
#export ROS_MASTER_URI=http://10.42.0.58:11311/
```
