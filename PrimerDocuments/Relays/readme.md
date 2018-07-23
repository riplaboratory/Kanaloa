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

## single-pole, single-throw, normally open (SPST NO) & single-pole, single-throw, normally closed (SPST NC)

The single pole, single throw relay is the most simple type of relay, and is appropriate for a simple ON-OFF switching application.  It has four total contacts, two for the coil (low-current trigger signal) side of the relay, and two for the switch (high current switch) side of the relay).  When a signal is sent to the coil side of the relay



# Relay Operation

A standalone relay generally has five contacts.
