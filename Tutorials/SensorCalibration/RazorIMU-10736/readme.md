# Razor 10736 IMU

As of 2019.03.23, the RIP Lab has three Razor 10736 IMU units.  Note that the hardware version (10736) is important, as there are multiple versions of this IMU.  It was purchased from this [Sparkfun listing](https://www.sparkfun.com/products/retired/10736).  Sparkfun keeps their legcy hardware archived on their website, but in case it is taken down for any reason, the documents are archived in this Git.  Other important links are: 

 - https://github.com/Razor-AHRS/razor-9dof-ahrs/wiki/Tutorial
 - https://github.com/sparkfun/9DOF_Razor_IMU
 
Note that the Github link is relevant for all versions of the Razor IMU.  When utilizing code from that Git, you must comment in which hardware revision you are using.  
 
## Calibration
 
To calibrate the IMU, follow the tutorial here: https://github.com/Razor-AHRS/razor-9dof-ahrs/wiki/Tutorial
 
When uploading firmware via the Arduino IDE, make sure you select the board `Arduino Pro or Pro Mini` with processor `ATmega328P (3.3v, 8MHz)`.  Also, in the code itself, make sure you comment in the line that selects which IMU hardware version you're using.  In our case it's the line that reads:

```
#define HW__VERSION_CODE 10736 // SparkFun "9DOF Razor IMU" version "SEN-10736" (HMC5883L magnetometer)
```

This is easy to miss in the tutorial if you're just looking at the calibration parts.
