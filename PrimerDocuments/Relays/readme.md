# Relays and Relay Modules

This is a primer document on the use of relays, and multi-channel relay board modules.  The purpose of a relay is to enable a user to switch a high current circuit (e.g. motor, light, heater, etc.) using a low-current trigger signal from a digital logic device (e.g. Arduino, Raspberry Pi, etc.).  This generally works by using a small electromagnetic coil to physically move a contactor between two poles inside the relay.  High-current relays are sometimes called "contactors" for this reason

From this point forward, we will refer to the high-current switching side of the relay as the "switch" side of the relay, and we will refer to the low-current trigger side of the relay as the "coil" side of the relay.

# Types of relay configurations

There are generally four different configurations of relays:

 - single-pole, single-throw (SPST)
   - single-pole, single-throw, normally-open (SPST NO)
   - single-pole, single-throw, normally-closed (SPST NC)
 - single-pole, double-throw (SPDT)
 - double-pole, single-throw (DPST)
   - double-pole, single-throw, normally-open (SPST NO)
   - double-pole, single-throw, normally-closed (SPST NC)
 - double-pole, double-throw (DPDT)
 

The **poles** refers to whether there is a switching circuit on one side of the switch (single-pole), or a switching circuit on both the both sides of the switch (double-pole).  The **throws** refers to whether the switching circuit simply breaks the connection (single-throw), or throws between two different current paths (double throw).  

## SPST NO & SPST NC (single-pole, single-throw, normally-open & single-pole, single-throw, normally-closed)

The SPST relay is the most simple type of relay, and is appropriate for a simple ON-OFF switching application.  It has four total contacts, two for the coil (low-current trigger signal) side of the relay, and two for the switch (high current) side of the relay.  When a signal is sent to the coil side of the relay, it moves a contactor to either connect (close), or break (open) the switch side of the relay.  

The normally-open, and normally-closed part of the relay refers to whether or not the switch side of the relay is open, or closed when the relay in its "normal" state (no current applied to the coil).  The vast majority of SPST relays are normally open, which means that the switch is open (off) when no current is applied to the coil.  This is because NO relays are the safer configuration for most systems (if the coil current were to fail for some reason, the switches system remains off); however, there are some cases where a NC SPST relay is desired.

A schematic of a SPST NO relay taken from [Electro Schematics](https://www.electroschematics.com/9593/normally-open-relay-switch/) is shown below:

![image](https://github.com/riplaboratory/Kanaloa/blob/master/PrimerDocuments/Relays/Images/SPST-NO-schematic.png)

A schematic of a SPST NC relay taken from [Electro Schematics](https://www.electroschematics.com/9595/normally-closed-relay-switch/) is shown below:

![image](https://github.com/riplaboratory/Kanaloa/blob/master/PrimerDocuments/Relays/Images/SPST-NC-schematic.png)

## SPDT (single-pole, double throw)

The SPDT relay is another common type of simple relay.  While a SPST relay can only switch a circuit on and off (open and close), the SPDT relay can swtich current between two different paths, e.g. when the coil is off, you can give power to one device, and when the coil is on, you can give power to a different device.  Disconnecting one of the two paths of a SPDT relay effectively turns it into a SPST relay.  A SPDT relay has five contacts, two for the coil (low-current trigger signal) side of the relay, and three for the double-throw switch (high current) side of the relay.

Because SPDT relays switch between two different possible current paths instead of simple opening (off) and closing (on) a switch, there is no distinction between normally-open (NO) and normally-closed (NC) SPDT relays; however, some SPDT relays have different current ratings between the two sides of the contactor.  In this case it will give a current rating for the switch in its NO state, and a current rating for the switch in its NC state.

A schematic of a SPDT relay taken from [Electro Schematics](https://www.electroschematics.com/9598/spdt-relay-switch/) is shown below:

![image](https://github.com/riplaboratory/Kanaloa/blob/master/PrimerDocuments/Relays/Images/SPDT-schematic.png)

## DPST NO & DPST NC (double-pole, single-throw, normally-open & double-pole, single-throw, normally-closed)

more in the future...
https://www.electroschematics.com/9605/dpst-switch-relay/

## DPDT (double-pole, double-throw)

more in the future...
https://www.electroschematics.com/9601/dpdt-switch-relay/


# Relay Boards/Modules


