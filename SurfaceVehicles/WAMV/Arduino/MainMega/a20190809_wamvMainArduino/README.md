Hacked the existing code to use the new centering circuit that requires 2 pwm pins per thruster.
one pin to command forward motion
one pin to command reverse motion
Assuming q1 is programmed to control a thruster mounted in the starboard front, 
the q1 channel signal is used to command the right thrusters
Assuming q2 is programmed to control a thruster mounted in the port front, 
the q2 channel signal is used to command the left thrusters

setSpd function added to control the switching of the channels and avoid shoot through.
