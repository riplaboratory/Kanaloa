The launch files in the this directoy are necessary to operate the WSN comms (see [SOP](https://github.com/riplaboratory/Kanaloa/tree/master/SurfaceVehicles/WSN/SOP) for instructions on running the system on the WAM-V).

`wsn_launch.launch` is already located on the WAM-V computer, but is in this directory for anyone wanting to run it on a different machine. 

`usb_cam-test.launch` is already located on the Raspberry Pi currently installed in the Camera Sensor Box, but is here in case the Pi needs replacement / the launch file needs to be used elsewhere. 

Note that all launch files were adpated from the launch file of each sensor's specific ROS driver (linked [here](https://github.com/riplaboratory/Kanaloa/tree/master/SurfaceVehicles/WSN/ROS%20Drivers)).
