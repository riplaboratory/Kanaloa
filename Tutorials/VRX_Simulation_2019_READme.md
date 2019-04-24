# Virtual RobotX Challenge (VRX) Simulation Instructions

  

  

**Document Version 1.1 as of 4/1/19, Matthew Bowers**

  

  

Aloha, and welcome to the Virtual Robot X competition! These are instruction on how to set up the simulation and properly record bag files. Changes from the past version of the simulation (vmrc) are noted. Note that while there are simulation tutorials that will be referenced, this document touches on operational things that the ROSwikis or tutorials may not provide intuition for.

  

  

# 1.) System Install

  

  

  

Follow the System Installation instructions found in the tutorials here: [System Install]([https://bitbucket.org/osrf/vrx/wiki/tutorials/SystemSetupInstall](https://bitbucket.org/osrf/vrx/wiki/tutorials/SystemSetupInstall)). The sourcing commands are as follows:

  

 

  

>source ~/Workspaces/vrx_ws/devel/setup.bash

  
  

  

**Tips**:

  

  

1. Note that the command for making a directory should have the filepath for where the vrx working directory has to go. If the directory is in the wrong spot, sourcing could cause errors.

  

  

2. Likewise, make sure the source to the bash is to the right directory. Each terminal tab opened needs to have the source command ran so it is good to get in the habit of sourcing things as you open new tabs in terminal.

  

  

# 2.) Sand Island Example Environment

  

  

Once ROS in installed and everything is properly sourced in a terminal tab, move to the the target workspace. If you are ready to begin taking sensor data into ROS, go ahead and build the operating system using the source commands. Then use the catkin_make command to build the environment. 

  

  

  

After sourcing and building ROS in a terminal the next goal is to test launching a model of the WAM-V in Gazebo.

  

  

If you are familiar with launching Gazebo in ROS, skip to the VRX section below. If not, use the tutorials here to become familiar with launching using Gazebo in ROS: [Sand Island Basic]([https://bitbucket.org/osrf/vrx/wiki/tutorials/Sand_Island_Basic](https://bitbucket.org/osrf/vrx/wiki/tutorials/Sand_Island_Basic))

  

  

  

# 3.) VRX Environment

  

  

The tutorials for getting a standard VRX environment launched is here: [VRX Example Environment]([https://bitbucket.org/osrf/vrx/wiki/tutorials/ExampleVrx](https://bitbucket.org/osrf/vrx/wiki/tutorials/ExampleVrx))

  

  

This is the environment that will be used to test the majority of the VRX tasks. Use the following command to launch:

  

  

>roslaunch vrx_gazebo vrx.launch

  

  

  

For the GPS, IMU, (potentially) Lidar, and cameras there will be some operations we do that are different from simply launching *localization_example.launch*. These differences will be addressed later when going over visualizing in RViz.

An image of the correct launch environment is shown in the VRX_Images folder on the Kanaloa github.


  

  

  

# 4.) Driving Methods

  

  

The tutorial for the different methods of driving the WAM-V is found here: [Driving Methods]([https://bitbucket.org/osrf/vrx/wiki/tutorials/Driving](https://bitbucket.org/osrf/vrx/wiki/tutorials/Driving))

  

  

The driving, though slightly unintuitive, is done by running any of the following commands in a new terminal tab in the target workspace.

  

  

For keydrive using the Teleop Keyboard package:

  

  

>roslaunch vrx_gazebo usv_keydrive.launch

  

  

and for driving using a gamepad use:

  

  

>roslaunch vrx_gazebo usv_joydrive.launch

  

  

I have only driven the WAM-V using the Teleop Keydrive Method. The link to the Keydrive Github package is here: [Teleop Package]([http://wiki.ros.org/teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard)). The instructions on the ROSwiki simply say 'moving around' without much hint as to what actually controls the [yaw]([https://en.wikipedia.org/wiki/Yaw_(rotation)) or how multiple thruster configurations are accounted for. It will take a little time messing around with the controls and getting used to the input delay from running a computationally taxing simulation.

  

  

The reason that a gamepad was not tested for steering is because the package only accepts inputs from several types of gamepads. Further details can be found here on the ROS wikis for the [joy teleop]([http://wiki.ros.org/joy_teleop](http://wiki.ros.org/joy_teleop)) and [joy]([http://wiki.ros.org/joy](http://wiki.ros.org/joy)) packages. Even though it would be complicated because Linux does its own controller stick displacement calculations implementing a universal tool like [X-padder]([https://softfamous.com/xpadder/](https://softfamous.com/xpadder/)) would allow any controller to steer the WAM-V.

An image of the correct launch environment and a correctly parked boat is shown in the VRX_Images folder on the Kanaloa github.

  

  

# Adding Course Elements via a World File

  

Here is the tutorial for adding course elements that will be elaborated on: [Adding Course Elements](https://bitbucket.org/osrf/vrx/wiki/tutorials/Adding%20course%20elements)

  

The tutorial says to make your own example_vrx_package directory but as long as you have a place that is in the workspace and can hold a copy of the example_course.world.xacro file you can use that directory. The command to copy the world file into the proper directory is :

  

> cd *directory you want the copied file in*

  

roscp vrx_gazebo example_course.world.xacro .

  

  

If unfamiliar, [this tutorial here](https://bitbucket.org/osrf/vrx/wiki/tutorials/Working%20with%20xacro%20files) job explaining what .xacros are. They are essentially MACROS for .xml files.

  

# Adding Sensors

  

The sensors used in our simulation and bag file gathering are USB cameras (insert name of camera here), GPS (insert name here), and an IMU (insert name here). [This is the tutorial](https://bitbucket.org/osrf/vrx/wiki/tutorials/AddingSensors) for adding sensors to our simulation. The 2 main types of files being worked with here are .urdf, [universal robot description format](http://wiki.ros.org/urdf), and .xacro, xml MACROs mentioned in the Course Elements section. The main thing to make note of is that the .xacro file must be converted to a .urdf using the ROS command:

  

>rosrun xacro xacro --inorder my_wamv.urdf.xacro -o my_wamv.urdf

  

  

The --inorder section of the command makes sure the conversion runs like how it did in the past version of ROS. If the --inorder command is not used then the resulting .urdf file will probably be blank. Here is a command to launch the simulation with the new custom .urdf:

  

>roslaunch vrx_gazebo sandisland.launch urdf:='pwd'/my_wamv.urdf

  

Here is a link to the [original .urdf file](https://cdn.discordapp.com/attachments/538325708754255884/546969340583608340/my_wamv.urdf) and here is the [list of sensor .xacro files](https://bitbucket.org/osrf/vrx/src/default/wamv_gazebo/urdf/sensors/) that are used in the simulation. As the original .urdf mentions, editing it by hand is not recommended. If editing any parameters is required, the parameters can be found in the .xacro file under their respective comments. For the simulation to accurately depict what would happen in the filed it is best to set the locations of the sensors in the simulation to match where they are placed on the WAMV.

  

Later sections will go over how to visualize the data coming in from sensors.

  
  

# Thruster Configurations

Here are the links for the [thruster configuration tutorials](https://bitbucket.org/osrf/vrx/wiki/tutorials/PropulsionConfiguration) and the [accompanying .xacro file](https://bitbucket.org/osrf/vrx/src/default/wamv_gazebo/urdf/wamv_gazebo.urdf.xacro). We only tested the WAMV with the thrusters in "T" configuration because that is the configuration we use in the field. The following lines of code are the ones that allow the thrusters to be implemented in the .xcaro file:

#

  

<!-- === Thrusters === -->

  

<xacro:property name="thruster_conf" value="$(arg thruster_config)"/> -->

  

  

<!-- Default WAM-V with two aft thrusters -->

  

<xacro:if value="${thruster_conf == 'H'}">

  

<xacro:wamv_gazebo thruster_layout="$(find wamv_gazebo)/urdf/thruster_layouts/wamv_aft_thrusters.xacro"/>

  

</xacro:if>

  

  

<!-- WAMV with "T" thruster configuration -->

  

<xacro:if value="${thruster_conf == 'T'}">

  

<xacro:wamv_gazebo thruster_layout="$(find wamv_gazebo)/urdf/thruster_layouts/wamv_t_thrusters.xacro"/>

  

</xacro:if>

  

  

<!-- WAMV with "X" thruster configuration -->

  

<xacro:if value="${thruster_conf == 'X'}">

  

<xacro:wamv_gazebo thruster_layout="$(find wamv_gazebo)/urdf/thruster_layouts/wamv_x_thrusters.xacro"/>

  

</xacro:if>

#

When launching the simulation using

> roslaunch vrx_gazebo sandisland.launch

  

it is possible to change the thruster configurations upon launch by adding another command in the line with the variable set to either "T" or "X", where"H" is the default used if there is no additional command upon launch:

> roslaunch vrx_gazebo sandisland.launch thrust_config:=T

  

# Visualizing With Sensors

Here is the [RVIZ visualizing tutorial](https://bitbucket.org/osrf/vrx/wiki/tutorials/Visualizing%20with%20RVIZ) that uses Gazebo in conjunction with RVIZ. In addition, Kazam is used to view bag files. Install Kazam by running the command:

> sudo apt install kazam