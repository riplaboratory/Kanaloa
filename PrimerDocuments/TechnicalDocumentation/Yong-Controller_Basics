# Controller Basics

### Introduction

This documentation discuss the basics of different types of controllers needed to make/operate a robot. There are two controllers that will be discussed: microcontroller and motor controller. In essence a microcontroller could be thought of as the control center of the robot because the responsibilities include: computations, decision making and communications. On the other hand a motor controller is an electronic device that is the "middle man" between the microcontroller, power supply, and the motors. The reason we need this "middle man" is because even though the microcontroller controls the actual robot it is very limited to power output. Therefore, a motor controller can provide the current and voltage required. The microcontroller and motor controller work side by side in order to make the motors on a robot work properly. 

### Microcontroller

![enter image description here](https://lh3.googleusercontent.com/5JjB3XFqYmViD90r3dCsyl3zv5Nzk0yaBsEFkDax6EomCklrJZC0ajOaHiY5DRTifBpemSLXowGy) 

Now how the microcontroller work are with a series of pins or signal connections. These pins send a signal which turns each pin ON(1 or HIGH) or OFF(0 or LOW). Each pin is to be determined by instruction from a program. 

Microcontrollers sound very basic and limited. However, that is not the case because there are almost infinite ways to program the pins with algorithms to achieve a mass variety of behaviors. Due to it being so versatile there are some limitations. Creating very complex and large algorithms for example vision processing would not be possible. This is due to the speed of the microcontrollers. Referring to a previous technical documentation: 

https://github.com/riplaboratory/Kanaloa/blob/technical-documentation/PrimerDocuments/TechnicalDocumentation/LightBuoyElectrical.md

In order to create a simple repeating light sequence the microcontroller has to turn the pins ON, wait then turn OFF and reapte to create a repeating effect. However, like previously stated a microcontroller has such a amazing versatility. They can be found in everyday electronic devices such as telephones, TV's, and robots. For Team Kanaloa it is obvious the microcontroller will control the speed and direction of the thrusters of the WAM-V. 

One may ask why choose a microcontroller over something more power such as an actual computer. Well there are obvious reasons such as size. Also unlike a CPU a microntroller do not need RAM or an external storage device. The upside for being less powerful the microcontrollers are more simpler to develop circuit and less expensive. Being less powerful also means the output is very weak. Since, the job of a microcontroller is to send signals and not actually power your project. 

### Motor controller

As discussed above a microcontroller does not have have enough power output. This is why a motor controller is required. A motor controller can provide the current at the required voltage but opposed to the microcontroller, the motor controller can not control speed/direction of an motor. This means the motor controller have to work simultaneously with the microcontroller to make the motors work. The microcontroller and motor controller communicate through a communication method called UART or PWM.  Team Kanaloa uses the PWM ([Link to more information regarding PWM)](https://learn.sparkfun.com/tutorials/pulse-width-modulation/all) method of communication. Just like microcontrollers there a variety of different motor controllers out there. Each with its own size, weight, and output. 

Team Kanaloa has decided to use Kelly Motor Controllers: 

![enter image description here](https://lh3.googleusercontent.com/SZgkkQjjFAIY7rXWlB1Z2I3G1mVkVYJylyQKYI9rD5jBAT1Ugg9egFklbuwyQFA4FML-mWPKpccz) 

[Link to the Kelly motor controllers user manual](http://kellycontroller.com/mot/downloads/KellyKDSUserManual.pdf)

### Conclusion

This concludes the basics of the controllers used for Team Kanaloa's WAM-V. Remember the most important part to get out of this documentation is the microcontroller acts as the "brain" or "control center" of the robot and the motor controller is the "middle man" for the microcontroller, power supply, and motors. Both need one another for the robot to operate. For more advanced documentation on controllers please look at technical documentations created by Daniel Truong ([Using the PWM shield](https://github.com/riplaboratory/Kanaloa/blob/technical-documentation/PrimerDocuments/TechnicalDocumentation/Using%20the%20PWM%20shield.pdf))
