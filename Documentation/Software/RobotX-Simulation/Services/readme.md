# Using Waypoint Navigation ROS Services in the VRX Simulation
Currently, the Station-keeping and Wayfinding tasks are both performed using the WAMV_Way_Point class. Both ROS services used by the WAMV_Way_Point class will need to be called in order to run station_keeping.py and wayfinding.py using manually inputted GPS coordinates. 

__Below is the procedure for calling the way_point and way_point_cmd services for the purpose of station-keeping or wayfinding.__

## Prerequisites

- Download [kanaloa_vrx workspace]  link is dead cuz I didn't upload the workspace to github yet
- VRX workspace, gazebo, generally things I need to link to JUdy and mArsa's tech doc

<hr>

## 1.Open the VRX Simulation in Gazebo
Open a terminal window and enter the following command:

`WORLD=stationkeeping0.world
roslaunch vrx_gazebo vrx.launch verbose:=true \
      paused:=false \
      wamv_locked:=true \
      world:=${HOME}/vrx_ws/src/vrx/vrx_gazebo/worlds/${WORLD} urdf:=$HOME/my_wamv/my_wamv.urdf`

We have set `WORLD` to be the station-keeping environment, but this can be changed to any other environment found in the following directory:

 `~/vrx_was/src/vrx/vrx_gazebo/worlds`

Note that the code used in this tutorial will only work with 
- a Holonomic or 'X' thruster configuration
- a sensor configuration that includes a GPS and IMU.


## 2.Run the script containing the WAMV_Way_Point class

In a new terminal tab, navigate to the kanaloa_vrx/src/kanaloa_pkg/scripts directory first, and enter the following command.

`python way_point_class_WAMV.py`

Upon pressing enter, you should see a continously printing statement similar to the one pictured below (NEED A PICTURE), and nothing should occur within the simulation.


## 3. Call the first service to add a waypoint

In a new terminal window, call `way_point_service`. Arguments are taken in this order: latitude, longitude, and station keeping time in minutes. Waypoints can be queued by calling this service multiple times with the desired waypoint coordinates.

Here is an example using a latitude of 21.3085, longitude of -157.8886, and desired station-keeping time of 10 minutes:

`rosservice call -- /way_point 21.31085 -157.8886 10`

###### Note: '--' is used so that negative arguments (such as -157.8886) are not interpreted as command line parsing.

Once called, you should see the service return a statement "recieved: True" and the print statement will now show the new queued coordinates and distance from them, as shown below. There will still be no movement in the simulation. (NEED A PICTURE)


## 4. Call the second service to manage navigation
You can stay within the same terminal window you used for Step 3, and call `way_point_cmd_service` in order to manage queued coordinates and navigation. The supported commands are:

__Start navigation:__

`rosservice call -- /way_point_cmd "Start Navigation"`

When this command is used, the WAMV will start navigating toward the given GPS coordinate in the Gazebo simulation.

__Stop Navigation:__

`rosservice call -- /way_point_cmd "Stop Navigation"`


__Remove the next queued coordinate:__

`rosservice call -- /way_point_cmd "Remove Next Coordinate"`


__Remove the coordinate the WAM-V is currently navigating towards:__

`rosservice call -- /way_point_cmd "Remove Current Coordinate"`


__Remove all queued coordinates:__

`rosservice call -- /way_point_cmd "Remove All Coordinates"`

<hr>

__Additional resources:__
- more info on [ROS services](http://wiki.ros.org/Services). 
- more info on [rosservice commands](http://wiki.ros.org/rosservice).
