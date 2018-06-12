# Persistent names for USB-Serial Devices (in Unix systems)

The concepts discussed in this tutorial are in major thanks to [Michael Ludvig at HintShop](http://hintshop.ludvig.co.nz/show/persistent-names-usb-serial-devices/).

The purpose of this tutorial is to assign persistent names to USB serial devices plugged into a Unix computer.  Normally when plugging in a serial device, it is assigned a generic name, e.g. `/dev/ttyUSB0` or `/dev/ttyACM1`, etc.  This can be problematic when using multiple USB devices, as we generally address these devices by these names.  It is thus, desirable to have persistent names assigned to the individual serial (the word 'serial' being used in the context of a manufacturer part number, as opposed to a 'serail'-type communication standard) number of a USB-serial device.  

First, unplug all of the USB-serail devices you wish to assign persistent names to.  Open a new terminal window, and type:

'''
lsusb
'''

