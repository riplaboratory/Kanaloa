# Master files

## Notice

12/9: 
12/8: Uploaded files used to test with imu, imu and gps. Everything works EXCEPT GPS INTEGRATION

12/4: added new files that hopefully fixed the GPS localization integration. Read instructions for usage

11/28: Code currently posts properly, needs to be tested with sensors attached. Uses GPS, IMU, LIDAR.

11/27: Work in progress, nothing works right now. Currently setting up transforms and making launch files easier to edit.

## About

This folder is a list of all launch files and a master launch file that will be used. It also includes a parameters.yaml file where all of the parameters for the launch files can be edited from one convenient location.

## Instructions

1. Copy files from launch and params try_this into local caktin/src/master respective folders.
2. There is no second IMU set up. Don't worry about it for now.
3. if the IMU TF fails to work, go in to razor-imu by roscd, nodes, then open imu_node.py. Search for 'frame' and check that the frame_id is the same as the frame listed in the transforms.launch file.
