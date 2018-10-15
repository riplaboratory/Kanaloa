# WAM-V ROS Info

## ROS Topics

| Topic Name | Message Type | Description |
| :---: | :---: | :---: |
| /autoQ1	| std_msgs/Float64	| The output desired for autonomous mode for the front right thruster. Set from -100 to 100. | 
| /autoQ2	| std_msgs/Float64	| The output desired for autonomous mode for the front left thruster. Set from -100 to 100. | 
| /autoQ3	| std_msgs/Float64	| The output desired for autonomous mode for the back left thruster. Set from -100 to 100. |
| /autoQ4	| std_msgs/Float64	| The output desired for autonomous mode for the back right thruster. Set from -100 to 100. |
| /diagnostics | diagnostic_msgs/Diagnostic | Array	collect information from hardware drivers and robot hardware to users and operators for analysis, troubleshooting, and logging.  |
| /Q1	| std_msgs/UInt16	| The output force of the front right thruster in lbf. Range from -55 to 55. |
| /Q2	| std_msgs/UInt16	| The output force of the front left thruster in lbf.  Range from -55 to 55. |
| /Q3	| std_msgs/UInt16	| The output force of the back left thruster in lbf. Range from -55 to 55. |
| /Q4	| std_msgs/UInt16	| The output force of the back right thruster in lbf.  Range from -55 to 55. |
| /rosout	| rosgraph_msgs/Log	| Standard ROS topic for publishing logging messages.  |
| /rosout_agg	| rosgraph_msgs/Log	| Aggregated feed of messages published to /rosout.  |
| /tf_static | tf2_msgs/TFMessage	| and transform on this topic is considered true at all times |
| /time_reference | sensor_msgs/TimeReference 	| Measurement from an external time source not actively synchronized with the system clock. |
| /vel | geometry_msgs/TwistStamped	| Velocity output from the GPS device. Only published when the device outputs valid velocity information. The driver does not calculate the velocity based on only position fixes.  |
| /voltMain	| std_msgs/Float64	| Volatage monitoring system. |
| /wamvAftCam/usb_cam/camera_info	| sensor_msgs/CameraInfo	| Height, width  distortion model; Matricies: D, K, R, P; binning_x, binning_y,  roi parameters |
| /wamvAftCam/usb_cam/image_raw	| sensor_msgs/Image	| The image topic for the USB camera. Uncompressed image  |
| /wamvAftCam/usb_cam/image_raw/compressed	| sensor_msgs/CompressedImage	| Compressed transport subtopic |
| /wamvDownCam/usb_cam/camera_info	| sensor_msgs/CameraInfo  | Height, width  distortion model; Matricies: D, K, R, P; binning_x, binning_y,  roi parameters |
| /wamvDownCam/usb_cam/image_raw	| sensor_msgs/Image | The image topic for the USB camera. Uncompressed image  |
| /wamvDownCam/usb_cam/image_raw/compressed	| sensor_msgs/CompressedImage | Compressed transport subtopic |
| /wamvFrontCam/usb_cam/camera_info	| sensor_msgs/CameraInfo | Height, width  distortion model; Matricies: D, K, R, P; binning_x, binning_y,  roi parameters |
| /wamvFrontCam/usb_cam/image_raw	| sensor_msgs/Image | The image topic for the USB camera. Uncompressed image  |
| /wamvFrontCam/usb_cam/image_raw/compressed	| sensor_msgs/CompressedImage | Compressed transport subtopic |
| /wamvGps | sensor_msgs/NavSatFix	| GPS Lon Lat Alt and covariance from nmea_navsat_driver package; remapped fromg /gps/fix |
| /wamvImu/data	| sensor_msgs/Imu	| Filtered orientation, accelerations and angular rotations: orientation is specified as a quaternion. remapped from imu/data. |
| /wamvImu/mag	| geometry_msgs/Vector3Stamped	| Filtered magnetometer data from sensor. provided as 3D orientation in X,Y,Z. remapped from imu/mag |
| /wamvImu/rpy	| geometry_msgs/Vector3Stamped	| Roll, Pitch and Yaw angles for sensed orientation: angles are specified in radians.  remapped from imu/rpy |
| /wamvImu/temperature	| std_msgs/Float32	| Temperature of sensor device in degrees centigrade. remapped from imu/temperature  |
| /wamvMmrCam/usb_cam/camera_info	| sensor_msgs/CameraInfo	| Height, width  distortion model; Matricies: D, K, R, P; binning_x, binning_y,  roi parameters |
| /wamvMmrCam/usb_cam/image_raw	| sensor_msgs/Image	| The image topic for the USB camera. Uncompressed image | /wamvMmrCam/usb_cam/image_raw/compressed	| sensor_msgs/CompressedImage	| Compressed transport subtopic |
| /wamvPortCam/usb_cam/camera_info	| sensor_msgs/CameraInfo	| Height, width  distortion model; Matricies: D, K, R, P; binning_x, binning_y,  roi parameters |
| /wamvPortCam/usb_cam/image_raw	| sensor_msgs/Image	| The image topic for the USB camera. Uncompressed image  |
| /wamvPortCam/usb_cam/image_raw/compressed	| sensor_msgs/CompressedImage	| Compressed transport subtopic |
| /wamvStarboardCam/usb_cam/camera_info	| sensor_msgs/CameraInfo	| Height, width  distortion model; Matricies: D, K, R, P; binning_x, binning_y,  roi parameters |
| /wamvStarboardCam/usb_cam/image_raw	| sensor_msgs/Image	| The image topic for the USB camera. Uncompressed image  |
| /wamvStarboardCam/usb_cam/image_raw/compressed	| sensor_msgs/CompressedImage	| Compressed transport subtopic |
