#PID Controller within the VRX Simulation

It was concluded that PID control would not be necessary in the Gazebo simulation since navigation was sufficient with just the existing P controller. In simulation, the motor dynamics are stronger than the inertial movement of the WAM-V, and did not severely affect navigation. 
As of now, implementation of a PID with the navigation script will not be implemented.


Should there be a need for virtual PID control in the future, this [resource](https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops)
was found for manually tuning a PID. 
