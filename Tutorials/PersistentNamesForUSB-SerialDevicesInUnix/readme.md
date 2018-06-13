# Persistent names for USB-Serial Devices (in Unix systems)

The concepts discussed in this tutorial are in major thanks to the following references:

   - [Michael Ludvig at HintShop](http://hintshop.ludvig.co.nz/show/persistent-names-usb-serial-devices/)
   - [A Stack Exchange question](https://unix.stackexchange.com/questions/66901/how-to-bind-usb-device-under-a-static-name)
   - [Another Stack Exchange question](https://unix.stackexchange.com/questions/378690/udev-rules-for-video-devices)
   - [Tayyar Guzel at Embedded Related](https://www.embeddedrelated.com/showarticle/1053.php)
   - [Daniel Drake at Reactivated](http://www.reactivated.net/writing_udev_rules.html)

The purpose of this tutorial is to assign persistent names to USB serial devices plugged into a Unix computer.  Normally when plugging in a serial device, it is assigned a generic name, e.g. `/dev/ttyUSB0` or `/dev/ttyACM1`, etc.  This can be problematic when using multiple USB devices, as we generally address these devices by these names.  It is thus, desirable to have persistent names assigned to the individual serial (the word 'serial' being used in the context of a manufacturer part number, as opposed to a 'serial'-type communication standard) number of a USB-serial device.  

## Finding the Vendor ID, Product ID, and serial number of your USB-Serial device

First, unplug all of the USB-serial devices you wish to assign persistent names to.  Open a new terminal window, and type:

```
lsusb
```

This lists all of the USB devices on your computer, with a Vendor ID and Product ID (in the form VendorID:ProductID).  Next, plug in the device you're interested in assigning a persistent name to, wait a few seconds, and type `lsusb` again.  Look for the device that changed from the last list of USB devices, and you now have the Vendor ID, and Product ID of the device you're interested in.  Hold on to this information.

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

At this point, you should have the Vendor ID, Product ID, and (optinally) the serial number of the device(s) you're interested in assigning a persistent name to.  To do this, we will create a `UDEV` ruleset for each of these new devices.  Your `UDEV` rules are usually scattered in many files in `/etc/udev/rules.d` (yes `rules.d` is a directory, __not__ a file).

We want to navigate to this directory and create a new file.  The easiest way to do this is by opening a terminal window and typing:

```
cd /etc/udev/rules.d
sudo gedit
```

This will open a gedit window.  In file, you can add new symbolic links by adding new lines containing information about the symbolic link.

__If the device you're interested in is a `tty` serial device, use this format to create a symbolic link:__

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="[VENDOR_ID]", ATTRS{idProduct}=="[PRODUCT_ID]", ATTRS{serial}=="[SERIAL_NUMBER]", SYMLINK+="[DESIRED_SYMBOLIC_NAME]"
```
__If the device you're interested in is a `video` serial device, use this format to create a symbolic link:__

```
KERNEL=="video*", ATTRS{idVendor}=="[VENDOR_ID]", ATTRS{idProduct}=="[PRODUCT_ID]", ATTRS{serial}=="[SERIAL_NUMBER]", SYMLINK+="[DESIRED_SYMBOLIC_NAME]"
```

Note that you must change the bracketed fields as appropriate to your device:

   - `[VENDOR_ID]` should be the four digit vendor ID you found earlier.
   - `[PRODUCT_ID]` should be the four digit product ID you found earlier.
   - `[SERIAL_NUMBER]` should be the serial number you found earlier (omit if not necessary).
   - `[DESIRED_SYMBOLIC_NAME]` should be the persistent name you wish to assign to the device.

An example for two video devices given the names 'cameraLeft' and 'cameraRight' is:

```
KERNEL=="video*", ATTRS{idVendor}=="12ab", ATTRS{idProduct}=="34cd", ATTRS{serial}=="1234ABCD", SYMLINK+="cameraLeft"
KERNEL=="video*", ATTRS{idVendor}=="12ab", ATTRS{idProduct}=="34cd", ATTRS{serial}=="5678EFGH", SYMLINK+="cameraRight"
```

Save and close your file.  To commit your changes, go back to your terminal and type:

```
sudo udevadm trigger
```

This will commit your new rules.  To check that your rules worked, you can use `ls /dev`.  When you plug in your device, it will still generate a generic `tty` or `video` directory in `/dev`; however, it will also generate a symbolic link (matching your desired name), which points to the correct generic directory.  Instead of referencing a generic link, you can now reference the symbolic link you created to always reference the correct USB device (as long as it's plugged in and detected).

You can explicitly check that your symbolic link works by typing into your terminal window:

```
ls -l /dev/[DESIRED_SYMBOLIC_NAME]
```

Which should output the generic directory associated with your persistent symbolic link.
