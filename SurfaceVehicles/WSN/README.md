# Wireless Sensor Nodes (WSN)

This directory contains software documentation for the Wireless Sensor Node (WSN) project, researched and developed by the F20-S21 Senior Design Team. Standard of Procedure to get WSN running from the WAM-V computer is in the SOP directory. 

<img width="1040" alt="Screen Shot 2021-05-11 at 12 19 47 PM" src="https://user-images.githubusercontent.com/43556054/117891755-2fdd6400-b253-11eb-99b5-da9334a73471.png">


## Purpose of WSN
The WSN system was designed in order to combat the complex wire management system onboard the WAM-V created by having wired sensors. Therefore, the wireless sensors node boxes (one for a ZED-F9P GPS, a BNO055 IMU, and a Logitech C920 HD Pro Webcam) each house a sensor that communicates data to the WAM-V's router through WiFi using ESP32 microcontrollers and a Raspberry Pi 3B+ (See the figure below for the software flow block diagram of the communication subsystem.) Though the system was designed and tested for the WAM-V, there is potential to adjust the design for Team Kanaloa's other surface vehicles.

<img width="1258" alt="Screen Shot 2021-05-11 at 12 14 04 PM" src="https://user-images.githubusercontent.com/43556054/117891312-636bbe80-b252-11eb-84f5-3ee69602cfa0.png">

## Further Work for WSN
One issue that was encountered was the inability to have all three sensors transmitting data at the same time when the WAM-V was tested at Sand Island. It is suspected that there were possible interference between TCP ports, and WSN was only ever deployed when there was also other sensors (outside of WSN) being tested. It is speculated that the second ROS Master these sensors were running with created this interference.

Another thing that should be visited is the use of the BNO055 IMU instead of the ADIS 16470 IMU. The WAM-V currently uses the ADIS, but there were issues having this work with the ESP32. More [here](https://drive.google.com/drive/u/1/folders/1SWtn3wieNyZFy6KR3JuZPfmqaPyLAWL3).


## More Documentation and Resources
The project was split into three subsystems: Mechanical, Electrical, and Communications. There was repeatable mounting (Mechanical) and independent and renewable power systems (Electrical) implemented for each sensor.  Documentation for the Electrical and Mechanical subsystems of WSN (as well as more resources used by Communications) can be found here: 

[Drive Documentation](https://drive.google.com/drive/u/1/folders/1-ApjlZHGvEL8MjFoPhoDieKhJmRhKBXQ)

The F20-S21 team website can also be found here:
[WSN Team Website](http://rip.eng.hawaii.edu/research/unmanned-x-systems/team-kanaloa-wireless-sensor-nodes/)



