# Kanaloa GNC Package

This package contains the ROS code and configurations which are use to implement GNC autonomoy on the WAMV, especially in the VRX simulation. This setup is configured to run on an Ubuntu 18 machine which has the VRX simulation dependencies installed.

## Package Usage

#### VRX

To launch a custom URDF WAMV configuration in VRX simulation, edit the `urdf` argument in the `vrx_custom_urdf.launch` file to match the URDF you want to use. Then launch the simulation:

```bash
roslaunch kanaloa vrx_custom_urdf.launch
```

#### robot_localization


#### move_base





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
