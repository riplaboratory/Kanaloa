# Adafruit Ultimate GPS (Breakout version)

This is a quick tutorial for interacting with the Adafruit Ultimate GPS.  This tutorial is for the breakout version.  There is another version of the GPS in the form of an Arduino shield.  The instructions for its use is subtly different.  

Original tutorial from here: https://learn.adafruit.com/adafruit-ultimate-gps?view=all

## Prerequisites

 1. Ubuntu 18.04
 2. Arduino IDE (or some kind of serial monitor, the Arduino one is convenient because you probably already have it, but there are certainly lighterweight options out there). 
 3. The GPS, and a FTDI breakout board.
 
## Instructions

 1. Connect the GPS to the FTDI as instructed here (https://learn.adafruit.com/adafruit-ultimate-gps?view=all).  Connect VCC (from FTDI) to VIN (to GPS); connect GND (from FTDI) to GND (to GPS); connect TXD (from FTDI) to RX (to GPS); connect RXD (from FTDI) to TX (to GPS).  
 
 2. Connect FTDI breakout to computer USB port.  Then type:
 
 ```ls /dev```
 
 This will bring up the list of connected devices in Linux.  If you have no other USB devices plugged in, this will probably be listed in `ttyUSB0`.  If you're having trouble figuring it out, unplug the device, type `ls /dev`, then plug it in, type `ls /dev` again, and look at the difference.  
 
 3. Open the Arduino IDE (or whichever serial monitor software you want).  Open the port you found in the previous step.  You may also have to set the baud rate.  
