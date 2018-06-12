# Persistent names for USB-Serial Devices (in Unix systems)

The concepts discussed in this tutorial are in major thanks to [Michael Ludvig at HintShop](http://hintshop.ludvig.co.nz/show/persistent-names-usb-serial-devices/).

The purpose of this tutorial is to assign persistent names to USB serial devices plugged into a Unix computer.  Normally when plugging in a serial device, it is assigned a generic name, e.g. `/dev/ttyUSB0` or `/dev/ttyACM1`, etc.  This can be problematic when using multiple USB devices, as we generally address these devices by these names.  It is thus, desirable to have persistent names assigned to the individual serial (the word 'serial' being used in the context of a manufacturer part number, as opposed to a 'serial'-type communication standard) number of a USB-serial device.  

## Finding the Vendor ID, Product ID, and serial number of your USB-Serial device

First, unplug all of the USB-serial devices you wish to assign persistent names to.  Open a new terminal window, and type:

```
lsusb
```

This lists all of the USB devices on your computer, with a Vendor ID and Product ID.  Next, plug in the device you're interested in assigning a persistent name to, wait a few seconds, and type `lsusb` again.  Look for the device that changed from the last list of USB devices, and you now have the Vendor ID, and Product ID of the device you're interested in.  Hold on to this information.

If all of the USB devices you use are different, then this is enough information to uniquely identify all of your USB devices; however, if you are using mmultiple of the *same* USB device (e.g. multiple Arduinos, cameras, FTDI breakouts (*even if the devices plugged into the breakout are different!*)), then you also need a *serial number* to distinguish these.  To find this information, unplug the device you're interested in, open a new terminal window, and type:

```
ls /dev
```

This displays all of the active devices in your `/dev` directory.  Next, plug in the device you're interested in back in, wait a few seconds, and type `ls /dev` again.  Look for the device directory name that changed from the last list of device directories.  Open a new terminal window and type:

```
udevadm info -a -n /dev/[DIERCTORY NAME] | grep '{serial}' | head -n1
```

Note that you must change `[DIRECTORY NAME]` to the name of your device directory name.  As an example, let's say that this is device `video1` (for a USB camera).  You should then type:

```
udevadm info -a -n /dev/video1 | grep '{serial}' | head -n1
```

The terminal should then display the serial number of your USB-serial device.  Hold on to this information.  

## Assigning persistent names




