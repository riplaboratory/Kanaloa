# usb_cam Launch Files

These launch files should go into `~/catkin_ws/src/usb_cam/launch`.  They can be launched with the command:

```roslaunch usb_cam launchFilename.launch```

## Some notes about usb_cam

### Checking which video device to use
To check which camera (video device) you should be using, open a new terminal window, and type:

```ls /dev```

This will bring up all of the external devices in your computer.  Look for a device called `video0`.  Each new uvc-compliant camera you have plugged into your computer will show up as a video device.  If you are using a laptop, you probably already have a video device as your built-in webcam (this is *usually* video0, but not always!).

Type `ls /dev`, *then* plug un your external uvc-camera, then type `ls /dev` again, and look for a new video device number.  Whichever one changed from the original output is the video device number Ubuntu has assigned to your external uvc-camera.  Make sure you use this camera number in your launch file.  

### Checking which resolution and pixel format you can use
Although the usb_cam node will let you select any arbitrary resolution, a typical uvc-compliant camera is only compatible with specific resolutions and pixel formats.  If you select a resolution for a camera that does support that resolution (even resolutions that are *less* than the native resolution of the sensor), it will run, but the camera will output garbage.  To check which options you have available to you, plug in your uvc-compliant camera, open a new terminal window, and type:

```
lsusb -v
```

This will output a lot of information.  Scroll until you find a device with multiple `VideoStreaming Interface Descriptor`.  Each field will give you information about an operation mode of that uvc-compliant device.  If you are using a laptop with a built-in webcam, it may be difficult to distinguish between what information is for the built-in webcam, and which information is for your external uvc-compliant camera.  If you're having trouble, you can unplug the external uvc-compliant camera, and look at the difference between the two outputs.  

Look for the fields `bDescriptorSubtype`, which tells you the pixel format to use (usually mjpeg), `wWidth`, which tells you the horizontal resolution, and `wHeight`, which tells you the vertical resolution.
