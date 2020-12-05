# Kanaloa GNC Package

This package contains the ROS code and configurations which are use to implement GNC autonomoy on the WAMV, especially in the VRX simulation. This setup is configured to run on an Ubuntu 18 machine which has the VRX simulation dependencies installed.

## Package Usage

#### VRX

To launch a custom URDF WAMV configuration in VRX simulation, edit the `urdf` argument in the `vrx_custom_urdf.launch` file to match the URDF you want to use. Then launch the simulation:

```bash
roslaunch kanaloa vrx_custom_urdf.launch
```

#### robot_localization

The robot_localization ROS package is used to fuse sensor data, primarily GPS and IMU data, to predict the current state of the robot. In this case, used to project the position, orientation, and linear/angular velocity of the WAMV.  

The robot_localization configuration can be seen in `vrx/wamv_gazebo/localization_example.launch` (Not in this package), and can be launched by the following in terminal:

```bash
roslaunch wamv_gazebo localization_example.launch
```

#### move_base

The move_base ROS package is used to help with the Path Planning of the robot. This packages utilizes the state of the WAMV from the robot_localization package, and receives a geometry_msgs/PoseStamped message from either the user or the Behavior Planning node of what the desired state of the robot is. This package then outputs a geometry_msgs/Twist message, which contains the velocity information (linear and angular) that the WAMV should immediately try to achieve in order to follow the generated path to the destination. This velocity should then be passed on to the WAMV base thruster controller.   

The move_package also uses a map of the environment (using the gmapping ROS package) in order to plan a path which avoids any obstacles taking into account the robot's size and the objects in the map. The map is generated using the information from the WAMV's LiDAR, which gets converted to a laser scan to generate a 2D map.

There are a series of configuration files specific to the WAMV used in simulation. These config files can be seen and modified in the `config/move_base` folder. 

To run move_base, gmapping, and pointcloud_to_laserscan ROS packages, just run the following launch files.

```bash
roslaunch kanaloa move_base.launch
roslaunch kanaloa mapping.launch
roslaunch kanaloa pointcloud_to_laserscan.launch
```


## Dependencies

Be sure to install the following dependencies
- VRX Dependencies
```bash
sudo apt update
sudo apt full-upgrade
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
DIST=melodic
GAZ=gazebo9
sudo apt install cmake mercurial git ruby libeigen3-dev ${GAZ} lib${GAZ}-dev pkg-config python ros-${DIST}-gazebo-plugins ros-${DIST}-gazebo-ros ros-${DIST}-hector-gazebo-plugins ros-${DIST}-joy ros-${DIST}-joy-teleop ros-${DIST}-key-teleop ros-${DIST}-robot-localization ros-${DIST}-robot-state-publisher ros-${DIST}-joint-state-publisher ros-${DIST}-rviz ros-${DIST}-ros-base ros-${DIST}-teleop-tools ros-${DIST}-teleop-twist-keyboard ros-${DIST}-velodyne-simulator ros-${DIST}-xacro ros-${DIST}-rqt ros-${DIST}-rqt-common-plugins protobuf-compiler
```

- ROS Planning Dependencies
```bash
sudo apt install libsdl-image1.2-dev
sudo apt install libsdl-dev
```

- TF Sensor ROS Messages
```bash
sudo apt install ros-melodic-tf2-sensor-msgs
```

- pointcloud_to_laserscan ROS package
```bash
sudo apt install ros-melodic-pointcloud-to-laserscan
```

- If unable to run Gazebo, make sure libignition is correct version
```bash
sudo apt update libignition-math2
```

- If running in a virtual machine, turn off "Accelerate 3D Graphics" in VM settings and run in terminal
```bash
export SVGA_VGPU10=0
```


Be sure to have the following ROS packages installed in order to use this package.

- navigation
- navigation_msgs
- robot_localization
- move_base
