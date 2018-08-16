# MOSFETs

A MOSFET (metal-oxide-semiconductor field-effect transistor) is a type of field effect transistor (FET) that is used for amplifying or switching electronic signals. 

A MOSFET has three leads, the gate, drain, and source.  By changing the voltage potential at the gate, the MOSFET generates an electrical field, which varies the resistance, and therefore, current flow between the drain and the source.  There are two types of MOSFETs: N-channel and, P-channel, as shown in the figure taken from the [Mechatronics/Robotics Lab at the National Institute of Technology, Calicut](http://www.rignitc.com/mosfets/).

![image](https://github.com/riplaboratory/Kanaloa/blob/master/PrimerDocuments/MOSFETs/Images/N_P_channelMosfets.png)

 - In the N-channel MOSFET (arrow points at the gate), the transistor is doped with more electrons, which results in current flow from the drain to the source.  As a result, with an N-channel MOSFET, you typically want to have the source pin connected to ground, and the drain pin is connected to your load.
 - In the P-channel MOSFET (arrow points away from the gate), the transistor is doped with more holes, which results in current flow from the souce to the drain.  As a result, with a P-channel MOSFET, you typically want to have the source pin connected to the load, and the drain pin connected to ground.

This might seem backwards, but remember, electrons are negatively charged particles that flow from ground to the higher voltage potential, and holes are gaps in electrons that flow from the higher voltage potential to ground.  

In a MOSFET's normal state (0 V applied to the gate), the resistance across the drain to the source is very high, which is almost like an open circuit (no current flow).  When a voltage potential from the gate to the source (gate-to-ground for an N-channel MOSFET, gate-to-load for a P-channel MOSFET), the resistance across the drain to the source is reduced, allowing current to flow.

## MOSFETs for switching

MOSFET gates (the gate-to-source voltage) have very high input impedance, making them ideal for use as a high current switch directly controlled by a logic component like an Arduino.  

https://www.electronics-tutorials.ws/transistor/tran_7.html

## MOSFETs for motor direction control

http://www.robotroom.com/BipolarHBridge.html
