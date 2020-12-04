# Robo Local Test Package

This package is used to test the robot_localization ROS package on a wheeled cart with sensors mounted to it.

## Building And Sourcing Your Terminal

Be sure to place this package into the `src/` directory of a ROS catkin workspace. You will need to compile and source the workspace in order for other commands in this document to work.

```bash
cd path/to/catkin_ws
catkin_make
source devel/setup.bash
```

To configure your environment so this workspace is always sourced, you can add the source command in your `.bashrc` file. If not you will need to run the source command each time a new terminal is opened.


## Testing Robot Localization Package On The Cart

In the `launch/` folder there are a series of launch files which can help launching the robot sensor packages and robot_localization. The latest working use of these launch files is to be used with the specified USB Camera, GPS and IMU. An example of running the robot_localization would be utilizing the `ad_imu_data_collection.launch` launch file with the Analog Devices IMU, RTK GPS, and Camera. A bag file is also recorded when launched and will be saved to the Desktop. To run open a terminal and run:

```bash
roslaunch robo_local_test ad_imu_data_collection.launch
```

Make sure to either match the USB naming, or modify the names of the sensor launch files. Change the `device` argument in `ad_imu.launch` and `port` argument in `ad_imu_data_collection.launch`.



## URDF Model

The package also comes with a URDF model to provide the sensor to base link transforms. These transforms can be visualized in RViz using the TF and Robot Model visualizers.

## Cart Simulation

A simulation is also available to test the robot_localization on the cart with simulated GPS and IMU data. To launch the simulation run:

```bash
roslaunch robo_local_test simulation.launch
```

To make the cart move, publish to the available wheel topics.

#### Using A Hand-held Controller With The Simulation

For better and more human like control, a node was created to control the cart in simulation using an XBOX 360 (or similar) USB controller. To use the joystick controller run the following in separate terminals:

```bash
rosrun joy joy_node
```

```bash
rosrun robo_local_test cart_controller.py
```


## Dependencies

Be sure to have the following ROS packages installed in order to use this package.

- joy (for simulation only)
- robot_localization
- nmea_navsat_driver
- adi_driver (if using AD IMU)
- um7 (if using UM7 IMU)
- usb_cam
- ros_control (for simulation only)
