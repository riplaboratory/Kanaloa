#!/bin/bash

BAG_FILE=name.bag

mkdir csv_files
rostopic echo -b ${BAG_FILE} -p /robot_localization/odometry/filtered > csv_files/robot_local_odom_filtered.csv
rostopic echo -b ${BAG_FILE} -p /imu/data > csv_files/imu.csv
rostopic echo -b ${BAG_FILE} -p /imu/data/fixed > csv_files/imu_fixed.csv
rostopic echo -b ${BAG_FILE} -p /fix/fixed > csv_files/gps.csv
rostopic echo -b ${BAG_FILE} -p /robot_localization/gps/filtered > csv_files/robot_local_gps_filtered.csv
