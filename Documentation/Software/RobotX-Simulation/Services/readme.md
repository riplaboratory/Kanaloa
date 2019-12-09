# Using ROS Services for Station-keeping and Wayfinding
The Station-keeping and Wayfinding tasks are both performed using the WAMV_Way_Point class. To run station_keeping.py and wayfinding.py using manually inputted GPS coordinates, you will need to call both ROS services used by the WAMV_Way_Point class.
More information on ROS services can be found [here](http://wiki.ros.org/Services). 

## __1.Run the script__ 

Navigate to kanaloa_vrx/src/kanaloa_pkg/scripts directory first.

`python vrx_way_point_class_WAMV.py`


## __2. Call way_point_service to add a waypoint__

In a new terminal window, call `way_point_service`. Arguments are taken in this order: latitude, longitude, and station keeping time in minutes. Waypoints can be queued by calling this service multiple times with the desired waypoint coordinates.

Here is an example using a latitude of 21.3085, longitude of -157.8886, and desired station-keeping time of 10 minutes:

`rosservice call -- /way_point 21.31085 -157.8886 10`

###### Note: '--' is used so that negative arguments are not interpreted as command line parsing.

More info on rosservice commands can be found [here](http://wiki.ros.org/rosservice).


## __3. Call way_point_cmd_service__

The way_point_cmd service manages navigation using the following commands.

__Start navigation:__

`rosservice call -- /way_point_cmd "Start Navigation"`


__Stop Navigation:__

`rosservice call -- /way_point_cmd "Stop Navigation"`


__Remove the next queued coordinate:__

`rosservice call -- /way_point_cmd "Remove Next Coordinate"`


__Remove the coordinate the WAM-V is currently navigating towards:__

`rosservice call -- /way_point_cmd "Remove Current Coordinate"`


__Remove all queued coordinates:__

`rosservice call -- /way_point_cmd "Remove All Coordinates"`
