# Relays and Relay Modules

This is a primer document on the use of relays, and multi-channel relay board modules.  The purpose of a relay is to enable a user to switch a high current circuit (e.g. motor, light, heater, etc.) using a low-current trigger signal from a digital logic device (e.g. Arduino, Raspberry Pi, etc.).  This generally works by using a small electromagnetic coil to physically move a contactor between two poles inside the relay.  High-current relays are sometimes called "contactors" for this reason

From this point forward, we will refer to the high-current switching side of the relay as the "switch" side of the relay, and we will refer to the low-current trigger side of the relay as the "coil" side of the relay.

# Types of relay configurations

There are generally four different configurations of relays:

 - single-pole, single-throw (SPST)
   - single-pole, single-throw, normally-open (SPST NO)
   - single-pole, single-throw, normally-closed (SPST NC)
 - single-pole, double-throw (SPDT)
 - double-pole, double-throw (DPDT)
 - double-pole, single-throw (DPST)

The **poles** refers to whether there is a switching circuit on one side of the switch (single-pole), or a switching circuit on both the both sides of the switch (double-pole).  The **throws** refers to whether the switching circuit simply breaks the connection (single-throw), or throws between two different current paths (double throw).  

## single-pole, single-throw, normally-open (SPST NO) & single-pole, single-throw, normally-closed (SPST NC)

The SPST relay is the most simple type of relay, and is appropriate for a simple ON-OFF switching application.  It has four total contacts, two for the coil (low-current trigger signal) side of the relay, and two for the switch (high current switch) side of the relay).  When a signal is sent to the coil side of the relay, it moves a contactor to either connect (close), or break (open) the switch side of the relay.  

The normally-open, and normally-closed part of the relay refers to whether or not the switch side of the relay is open, or closed when the relay in its "normal" state (no current applied to the coil).  The vast majority of SPST relays are normally open, which means that the switch is open (off) when no current is applied to the coil.  This is because NO relays are the safer configuration for most systems (if the coil current were to fail for some reason, the switches system remains off); however, there are some cases where a NC SPST relay is desired.

A schematic of a SPST NO relay taken from [Electro Schematics](https://www.electroschematics.com/9593/normally-open-relay-switch/) is shown below:

![image](https://github.com/riplaboratory/Kanaloa/blob/master/PrimerDocuments/Relays/Images/SPST-NO-schematic.png)

A schematic of a SPST NC relay taken from [Electro Schematics](https://www.electroschematics.com/9595/normally-closed-relay-switch/) is shown below:

![image](https://github.com/riplaboratory/Kanaloa/blob/master/PrimerDocuments/Relays/Images/SPST-NC-schematic.png)


## single-pole, double throw (SPDT)

The single-pole

# Relay Operation

A standalone relay generally has five contacts.
