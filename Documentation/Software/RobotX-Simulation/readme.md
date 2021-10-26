# Installing RobotX Simulation (Virtual RobotX Challenge)

This tutorial assumes that you already have Ubuntu 18.04 and ROS Melodic installed on your machine. Team Kanaloa takes no credit for this project and this page should only be used as a reference by its members. Original publication credit goes to Brian Bingham, Carlos Aguero, Michael McCarrin, and the RobotX community. Their repository can be found [here](https://bitbucket.org/osrf/vrx/src/default/).

## Prerequisites

1. Ubuntu 18.04
2. ROS Melodic
3. Gazebo 9.11.0 or greater (<9.11.0 is not sufficient)

VRX recommends that your computing hardware has:
- Modern multi-core CPU, e.g. Intel Core i5
- 8 Gb of RAM
- Discrete Graphics Card, e.g. Nvidia GTX 650

#### Note
 - As of July 6, 2021, the original VRX repository has been migrated to compatibility for Ubuntu 20.04 and ROS Noetic. 
 - The following documentation will be using an old branch in order to work with the prerequisites stated above.
 - VRX with the prequisites above is now only community supported and not officially supported by OSRF

## Installing the simulation

This tutorial will walk you through the setup required to prepare a computer to run the VRX simulations. In order to run the VRX simulation, your computer will need a discrete graphics card and must satisfy the minimum System Requirements. All updates and official tutorials can be found here: ~~https://bitbucket.org/osrf/vrx/wiki/Home~~ https://github.com/osrf/vrx/wiki

These instructions contain information for building the VRX environment in Gazebo. All commands should be run via the command terminal in Ubuntu.

### 1. Update apt
Because the simulation uses some relatively new (as of winter 2019) features in ROS and Gazebo, it is highly recommended that you upgrade the packages installed on your system:
```
sudo apt update

sudo apt full-upgrade
```

### 2. Setup and install dependencies
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
DIST=melodic
GAZ=gazebo9
sudo apt install cmake mercurial git ruby libeigen3-dev ${GAZ} lib${GAZ}-dev pkg-config python ros-${DIST}-gazebo-plugins ros-${DIST}-gazebo-ros ros-${DIST}-hector-gazebo-plugins ros-${DIST}-joy ros-${DIST}-joy-teleop ros-${DIST}-key-teleop ros-${DIST}-robot-localization ros-${DIST}-robot-state-publisher ros-${DIST}-rviz ros-${DIST}-ros-base ros-${DIST}-teleop-tools ros-${DIST}-teleop-twist-keyboard ros-${DIST}-velodyne-simulator ros-${DIST}-xacro ros-${DIST}-rqt ros-${DIST}-rqt-common-plugins protobuf-compiler
```

### 3. Build VRX from source
#### 3.1 Create a catkin workspace for VRX. 
If you are familiar with ROS catkin workspaces, this is a similar concept. Here, you will create a new catkin workspace called `vrx_ws`.
```
mkdir -p ~/vrx_ws/src
```
Navigate to your newly-created directory:
```
cd ~/vrx_ws/src
```
Clone the VRX repository:
~~hg clone https://bitbucket.org/osrf/vrx~~ 
```
git clone https://github.com/osrf/vrx.git
```
Navigate to the cloned repository:
```
cd ~/vrx_ws/src/vrx
```
Switch to the `noetic_migration` branch:
```
git checkout noetic_migration
```
#### 3.2 Build the software
First, source the ROS `setup.bash` file:
```
source /opt/ros/melodic/setup.bash
```
Then build all the software:
```
cd ~/vrx_ws
catkin_make
```
In order to run the VRX simulation, the VRX `setup.bash` file must be sourced every time you open a new terminal. Therefore, so that it is sourced automatically, it is recommended that you add the following line to your `.bashrc` file:
```
source  ~/vrx_ws/devel/setup.bash
```

### 4. Test the simulation 
Navigate to your catkin workspace, then launch the VRX simulation with a simple world.
```
roslaunch vrx_gazebo sandisland.launch
```

## Thruster Configurations

There are currently 3 supported Propulsion options for the WAM-V: 'H', 'T', and 'X'.  The 'H' configuration is a differential-drive thruster configuration with two fixed stern thrusters mounted in the surge direction; the 'T' configuration is also a differential-drive thruster configuration with two fixed stern thrusters mounted in the surge direction, but with an additional lateral/bow thruster mounted in the sway direction; the 'X' configuration contains four fixed thrusters, two thrusters in quadrants 1 & 2 rotated in the positive y direction inwards at 45 degrees and two thrusters in quadrants 3 & 4 rotated in the negative y direction inward at 45 degrees .  Each individual thruster has its own unique ROS topic name and can be manipulated through ROS scripts. The default thruster configuration upon launching a VRX simulation is the 'H' configuration. To utilize a different configuration a new parameter must be put in the roslaunch command.

![Image of Thruster Configurations](https://bitbucket-assetroot.s3.amazonaws.com/repository/BgXLzgM/2101300599-Propulsion%20Options.png?AWSAccessKeyId=AKIAIQWXW6WLXMB5QZAQ&Expires=1575932639&Signature=27joZ5gbaR3wK6iJ1uwntnF5mNs%3D)

To launch the basic simulation with the WAM-V in the default 'H' configuration, run:
```
roslaunch vrx_gazebo sandisland.launch
```
To launch the basic simulation with the WAM-V in the 'T' configuration, run:
```
roslaunch vrx_gazebo sandisland.launch thrust_config:=T
```
To launch the basic simulation with the WAM-V in the 'X' configuration, run:
```
roslaunch vrx_gazebo sandisland.launch thrust_config:=X
```
Tip: To see the thrusters in the water in Gazebo, in the toolbar, go to “View” and select “Wireframe.”

## Driving

### Option 1: Teleop_twist_keyboard 
This option was created by the developers of VRX and is convenient to use in order to quickly test other areas of the WAM-V. It only works with the default thruster configurations; if you have created a custom URDF by editing the YAML file and changed the thruster names, the topic names for the thrusters will not match and the WAM-V will not move. To start off, open the basic `sandisland.launch` file:
```
roslaunch vrx_gazebo sandisland.launch
```
To use the keyboard, we use the [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard) package, along with a custom [twist2thrust.py](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/nodes/twist2thrust.py) node to convert the Twist messages to two Float32 messages for the left and right thrusters. Forward velocity (twist.linear.x) is mapped to axial thrust (right+left) and rotational velocity (twist.linear.z) is mapped to differential thrust (usvdrive.right-usvdrive.left).
```
roslaunch robotx_gazebo usv_keydrive.launch
```
The basic setup from the example above only works for the `H` and `T` propulsion configurations. This is because the topic names for the thrusters in the holonomic 'X' configuration are different, due to having more thrusters in differing orientations. 

In order to drive the WAM-V in the `X` configuration, we need to change the ROS interface between the teleop nodes and the simulated WAM-V to include the extra rear modifier. The launch file accepts a command line argument for this case:
```
roslaunch vrx_gazebo usv_keydrive.launch thrust_config:=X
```

### Option 2: Kanaloa drive script from real-life WAMV
This option is for updating and editing code to specific thrusters, such as X, H, or T configurations. 

Code for Kanaloa WAMV drive script [here](https://github.com/riplaboratory/Kanaloa/blob/master/SurfaceVehicles/WAMV/Python/WAMV_drive_pub.py)

To edit thruster configurations, insert correct ROS topic lists and adjust speed.

## Creating a custom WAM-V thruster and sensor configuration

This tutorial demonstrates how to create a custom WAM-V Thruster and Sensor Configuration for Competition. This involves writing a user-generated thruster YAML file and a user-generated sensor YAML file, and then running a script that will generate a custom WAM-V URDF file with the specified thrusters and sensors. This WAM-V URDF file can then be passed in as a parameter to the VRX simulation roslaunch files. The script to generate a custom WAM-V URDF also makes sure that the requested thruster and sensor configurations are in compliance with basic VRX constraints, which will be explained in further detail below.

### Creating a URDF file
A URDF file is a format to describe a robot including joints, sensors, inertial properties, and more. The file is used by Gazebo, RViz, and many other ROS packages. Several example URDF files for representing a WAM-V are included in the VRX packages.

Create a directory for your custom WAM_V:
```
mkdir ~/my_wamv
```

Create a new file, which you will edit to customize the thruster configuration of your WAM-V:
```
gedit ~/my_wamv/thruster_config.yaml
```

Copy and paste the following example thruster configuration into the newly-created file:
```
engine:
  - prefix: "left"
    position: "-2.373776 1.027135 0.318237"
    orientation: "0.0 0.0 0.0"
  - prefix: "right"
    position: "-2.373776 -1.027135 0.318237"
    orientation: "0.0 0.0 0.0"

  # Adding new thruster
  - prefix: "middle"
    position: "0 0 0.318237"
    orientation: "0.0 0.0 0.0"
```
Note that compared to the [default thruster configuration](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/src/vrx_gazebo_python/generator_scripts/wamv_config/example_thruster_config.yaml), this example configuration has an additional thruster located in the middle of the WAM-V so that it uses the `T` configuration, rather than the default `H` configuration.

Create another file, which you will edit to customize the sensor configuration of your WAM-V:
```
gedit ~/my_wamv/sensor_config.yaml
```

Copy and paste the following example sensor configuration into the newly-created file:
```
wamv_camera:
    - name: front_left_camera
      x: 0.75
      y: 0.1
      P: ${radians(15)}
      Y: ${radians(270)}
    - name: front_right_camera
      x: 0.75
      y: -0.1
      P: ${radians(15)}
      Y: ${radians(90)}
wamv_gps:
    - name: gps_wamv
      x: -0.85
wamv_imu:
    - name: imu_wamv
      y: -0.2
wamv_p3d:
    - name: p3d_wamv
lidar:
    - name: lidar_wamv
      type: 16_beam
      P: ${radians(8)}
```
Note that compared to the [default sensor configuration](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/src/vrx_gazebo_python/generator_scripts/wamv_config/example_sensor_config.yaml), this configuration also has two front cameras, a GPS, IMU, p3d, and lidar, but the middle right camera has been removed and the left and right front cameras are facing port and starboard respectively.

Run the script to generate your WAM-V's URDF with these newly specified thrusters and sensors. Note: on most systems, `$HOME` is `/home/<username>`. If this is not the case, you can change all uses of `$HOME` to `/home/<username>`.
```
roslaunch vrx_gazebo generate_wamv.launch thruster_yaml:=$HOME/my_wamv/thruster_config.yaml sensor_yaml:=$HOME/my_wamv/sensor_config.yaml wamv_target:=$HOME/my_wamv/my_wamv.urdf
```
Parameters Explained:
- `thruster_yaml`: the input - the full path of the thruster YAML configuration file. If not given, the script uses the [default thruster yaml](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/src/vrx_gazebo_python/generator_scripts/wamv_config/example_thruster_config.yaml).
- `sensor_yaml`: the input - the full path of the sensor YAML configuration file. If not given, the script uses the [default sensor yaml](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/src/vrx_gazebo_python/generator_scripts/wamv_config/example_sensor_config.yaml).
- `wamv_target`: the output - the full path to the WAM-V URDF, which will be generated.

After running the script to generate your WAM-V’s URDF, you should see the following confirmation message in the terminal with no errors present:
```
[INFO] [1566845959.198003]:
Using /home/tylerlum/my_wamv/thruster_config.yaml as the thruster configuration yaml file

[INFO] [1566845959.208003]: 
Using /home/tylerlum/my_wamv/sensor_config.yaml as the sensor configuration yaml file

xacro: in-order processing became default in ROS Melodic. You can drop the option.

WAM-V urdf file sucessfully generated. File location: /home/tylerlum/my_wamv/my_wamv.urdf
================================================================================REQUIRED process [wamv_config/wamv_generator-2] has died!
process has finished cleanly
log file: /home/tylerlum/.ros/log/9b1fd6a0-c833-11e9-a434-dcfb48e97aeb/wamv_config-wamv_generator-2*.log
Initiating shutdown!
================================================================================
```

Launch the example world with your customized WAM-V configuration by inserting a command line argument:
```
roslaunch vrx_gazebo sandisland.launch urdf:=$HOME/my_wamv/my_wamv.urdf
```
The WAM-V should have the thrusters and sensors as specified in your YAML files. For a better view of the thrusters and sensors, you can click `View` > `Transparent` in the Gazebo toolbar.

### Compliance
The number of each thruster and sensor in the configuration are restricted to the limits defined [here for thrusters](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/src/vrx_gazebo_python/generator_scripts/wamv_config/thruster_compliance/numeric.yaml) and [here for sensors](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/src/vrx_gazebo_python/generator_scripts/wamv_config/sensor_compliance/numeric.yaml).

The bounding boxes of compliant YAML configuration files can be found [here](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/src/vrx_gazebo_python/generator_scripts/wamv_config/thruster_compliance/bounding_boxes.yaml) for thrusters and [here](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/src/vrx_gazebo_python/generator_scripts/wamv_config/sensor_compliance/bounding_boxes.yaml) for sensors. These bounding boxes indicate the possible locations (`x`, `y`, and `z` parameters) for your sensors and thrusters.

Note that for thrusters, there can only be one thruster in each bounding box. This is to prevent teams from stacking thrusters together in one location, which is physically infeasible.

If you call `generate_wamv.launch` on non-compliant configuration YAML files, red error messages will be printed but the URDF file will still be created and usable. However, it is not a valid configuration for the VRX competition.

### Parameters
All sensors and thrusters have numerous parameters. These include `name`, `prefix`, `position`, `orientation`, `x`, `y`, `z`, `R`, `P`, `Y`, `post_Y`, depending on the sensor or thruster. You can view the parameters for thrusters [here](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/src/vrx_gazebo_python/generator_scripts/wamv_config/thruster_compliance/bounding_boxes.yaml) and for sensors [here](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/src/vrx_gazebo_python/generator_scripts/wamv_config/sensor_compliance/bounding_boxes.yaml).

In the YAML files, if an `engine`, `wamv_camera`, `wamv_gps`, etc. is included, there are certain parameters which must be specified, while the rest of the parameters can be omitted (they will automatically be set to the default configurations if unspecified). For sensors, `name` is a required parameter which must be specified, and for thrusters, `prefix` is a required parameter. Changing `name` or `prefix` changes the name of the ROS topic to which the sensor’s or thruster’s data is published.

The `position`, `orientation`, `x`, `y`, `z`, `R`, `P`, and `Y` parameters define the exact pose of the sensor or thruster you want with respect to the WAM-V. `post_Y` is a parameter only used for the sensors and simply changes the angle at which the post is sitting underneath the given sensor. The angle by which the sensor or thruster rotates according to the `R`, `P`, `Y`, and `post_Y` parameters is converted from degrees to radians via a Python expression `radians()`.

## Visualization

[RViz](http://wiki.ros.org/rviz) is a standard ROS tool for visualizing messages. This tutorial explains how to run RViz to display the WAM-V and sensors that you are simulating with Gazebo.

### Launch Gazebo
You can run a simulation that is configured similarly to a typical RobotX "competition-ready" boat with the sample launch file:
```
roslaunch vrx_gazebo vrx.launch 
```
This example illustrates a specific configuration similar to what some of the RobotX teams used in 2018. It includes:
- Fixed stern thrusters and a single lateral thruster (the 'T' configuration)
- A standard set of onboard sensors
- Two forward facing cameras (stereo)
- One starboard facing camera
- GPS
- IMU
- 3D LiDAR

Leave this simulation running for the remainder of the tutorial. In the steps below, you will use the robot_state_publisher to make the simulated data available to RViz.

### Publish a TF tree
RViz depends on TF for understanding where to display everything. A TF tree is a package that lets the user keep track of multiple coordinate frames over time. If you want more information on what TF is, visit [here](http://wiki.ros.org/tf). 

This is usually accomplished with the `robot_state_publisher`. This program will publish all of the fixed joints in your URDF to the TF tree along with non-fixed joints based on messages published to /JointStates (like the propeller rotations).
There are two options for doing this:

#### Option 1: Using the default URDF
If you are using the example `wamv_gazebo.urdf`, you can simply run:
```
roslaunch wamv_gazebo localization_example.launch
```
This will run `robot_state_publisher` along with a localization node which fuses the GPS and IMU into the robot's global pose and publishes it to the odom frame. 

#### Option 2: Using custom URDF
If you are using a custom URDF without the standard GPS/IMU configuration, you can run the `robot_state_publisher` alone:
```
rosrun robot_state_publisher robot_state_publisher
```

### Run RViz
To open RViz with a configuration made for the WAM-V, open a third terminal and run:
```
roslaunch wamv_gazebo rviz_vrx.launch
```
RViz should open and display the WAM-V and a camera. Driving around using the driving teleop keyboard should cause the robot to move both in Gazebo and RViz.

### Troubleshooting
#### Change Fixed Frame
The example RViz config starts with "odom" set as the frame everything is displayed in. If you are using the example URDF with a GPS and IMU and running the localization_example.launch, this frame should exist. However, if you are using a custom URDF/localization which does not publish an odom frame, you can always switch the frame to base_link to display messages relative to the WAM-V:

![Image of Changing Fixed Frame](https://bitbucket-assetroot.s3.amazonaws.com/repository/BgXLzgM/636844775-rvizswitchframe.png?AWSAccessKeyId=AKIAIQWXW6WLXMB5QZAQ&Expires=1575940238&Signature=seXMi0ZlKA8Paqe1AawLSzuJzmE%3D)
