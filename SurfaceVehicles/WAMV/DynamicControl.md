# Development of Dynamic Control for the WAM-V

## Readings
*Handbook of Marine Craft Hydrodynamics and Motion Control*
[Link to the book on the Kanaloa drive](https://drive.google.com/open?id=19-EfR_QwlcKtepS_aFD7peaLzxZ65Tyy)
*Study on Maneuverability and Control of an Autonomous Wave Adaptive Modular Vessel (WAM-V) for Ocean Observation*
[Link to the paper on the Kanaloa drive] (https://drive.google.com/open?id=1TeaEEUGGvbEN9jk1v1Dhgn1CN401Ei4E)

This document describes the development of Dynamic Control for the WAM-V

Currently the WAM-V is controlled in a very simple manner. The thrust is varied by changing the voltage going to the thrusters.
A better way to control the  WAM-V would be dynamic control. 
Dynamic Control incorporates aspects of the WAM-V into the control of the robot through characterizations of the WAM-V. 
This includes things like the thruster configuration and the forces acting on the WAM-V.
Dynamic Control of the WAM-V is possible through characterizing the WAM-V.

The book, Handbook of Marine Craft Hydrodynamics and Motion Control by Thor Fossen, describes ways to characterize a vessel.
Specifically, the book mentions six (6) ship maneuvers to characterize a vessel.

The Maneuvers
- Turning Circle
- Kempf’s Zigzag Maneuver
- Pull-Out Maneuver
- Dieudonne’s Spiral Maneuver
- Bech’s Reverse Spiral Maneuver
- Stopping Trials

The turning circle is specifically interesting because it allows for something called the Nomoto Gain to be found.
The Nomoto Gain is a coefficient related to other equations proposed by Nomoto. 
The problem with the way the book presents it is that it relies on the rudder angle.
The WAM-V does not possess a rudder some alterations would need to be made to the equation in the book to be useful for the WAM-V.

The paper, Study on Maneuverability and Control of an Autonomous Wave Adaptive Modular Vessel (WAM-V) for Ocean Observation, 
delves into the dynamics and modeling of a WAM-V.
From the paper, an equation for the WAM-V motion response was found.
This equation was interesting as they described it as an extension to the 2nd order Nomoto equation.
The paper also describes the testing environment, with the WAM-V being in a tow tank.

The methods used to characterize vessels presented in the book and the paper wouldn’t work for our WAM-V.
The handbook describes methods that require a rudder and the paper characterized their WAM-V using a tow tank.
Our WAM-V does not have a rudder nor do we have access to a tow tank that can fit our WAM-V. 

This does serve as a good foundation for what to do next.
To test the maneuvers is through using the MRUH, in order to get an idea of what the data looks like in a real world setting.
This would require getting MRUH in a state where it’s ready to be used.
Another thing that needs to be done next is developing a better understanding of the math that went into the equations.
Understanding the math in the book and the paper and their relations would certainly help characterize the WAM-V.
Lastly, and most importantly, a way to test using the WAM-V needs to be figured out.
The testing methods shown in the book and paper won’t work exactly as is for our WAM-V, so another method would have to be found.
