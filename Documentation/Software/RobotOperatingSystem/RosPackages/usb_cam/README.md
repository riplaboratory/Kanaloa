# usb_cam instructions

`usb_cam` is a ROS node that acts as a driver for UVC (USB Video Class)-compliant device.  This includes most webcams, but to be sure, you should check that your external video device is actually UVC-compliant.  If it is not (uses proprietary drivers),  then this node will not work!

## Installation

usb_cam has an apt repository, so it may be installed using: 
 
```
sudo apt-get install ros-[ROS_VERSION]-usb-cam
```
 
Where [ROS_VERSION] is your version of ROS, e.g. `kinetic` or `melodic`.

### Alternative installation to workspace only

usb_cam is compliant with installing directly to your catkin workspace, so it may be installed using:

```
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/usb_cam.git
cd ~/catkin_ws
catkin_make
```
## Writing a launch file

### Setting up a `usb_cam` launch file
The `usb_cam` launch files are stored in `~/catkin_ws/src/usb_cam/launch`.  By default, there is a sample launch file called `usb_cam-test.launch` that you may use for reference; however, this launch file does not use all of the possible parameters.  A list of the possible parameters can be found [at the usb_cam wiki page](http://wiki.ros.org/usb_cam).

For our purposes, the important parameters to get correct are:

   - `video_device`
   - resolution (`image_width` and `image_height`)
   - `pixel_format`
   - `framerate`
   - `autofocus` and `focus`
 
There are other parameters, but these are the ones that are crucial to change from the defaults.  Note that the default values can be changed in the underlying C++ file, which is located in `~/catkin_ws/src/usb_cam/nodes/usb_cam_node.cpp`.

First, the `video_device` field tells the node which video device to use.  If you are using computer without an integrated webcam, this is most likely `/dev/video0`.  If your computer is equipped with an integrated webcam, then an external video device is *probably* `/dev/video1`.  If your external video device is plugged in when the computer power cycles however, the video device can switch spots, so be careful.  To know for certain, make sure your external video device is *unplugged*, open a terminal window and type:

```
ls /dev
```

Take a note of the `video` devices available (if your computer does not have an integrated webcam, then you probably won't see any `video` devices.  Plug your external video device in, and type `ls /dev` again.  Whichever new video device appears is your external video device.  For example, if this is `video1`, then your parameter line should read:

```
<param name="video_device" value="/dev/video1" />
```

For the resolution (`image_width` and `image_height`), `pixel_format`, and `framerate` parameters, it is important to note that you cannot necessarily enter arbitrary values--the external video device must explicitly support the parameters you enter or your may run into issues.  To check these details in a Unix environment, open a terminal window and type:

```
lsusb -v
```

This will display the details of all of the USB devices connected to the computer.  If your computer has one, it may be challenging to distinguish the details for your integrated webcam from your external video device.  If you cannot tell, you can employ the unplug-then-plug-in-and-look-at-the-change method we used earlier (except this time with the `lsusb -v` command).  

Look particularly for the field `VideoStreaming Interface Descriptor`, as this denotes a video device.  There should be multiple listings for a single video device.  The field `wWidth` gives you the supported horizontal resolution, the field `wHeight` gives you the supported vertical resolution.  You must choose one of these supported resoution pairs.  Additionally, the field `bDescriptorSubtype`, gives you information about the supported `pixel_format` of the video device.  `(FORMAT_UNCOMPRESSED)`, means that the device is compatible with the `yuyv` pixel format, whereas `(FORMAT_MJEPG)` is compatible with the `mjpeg` pixel format.  Finaly, the field `dwFrameInterval` gives you information about the supported framerates.  This number is given as a period in [s E-7] for some reason, e.g. 30 [fps] = 333333 [s E-7]; 60 [fps] = 166666 [s E-7].  You should choose a framerate that matches one of these values (in [fps]).

For example, for an external video device with resolution 1920x1080, that supports `(FORMAT_UNCOMPRESSED)`, and a framerate of 60 fps, your parameters would look something like:

```
<param name="image_width" value="1920" />
<param name="image_height" value="1080" />
<param name="pixel_format" value="yuyv" />
<param name="framerate" value="60" />
```

Finally, for `autofocus` and `focus`, for applications that involve a human looking through a camera, it is probably better to leave `autofocus` as `true` and omit the `focus` parameter.  For applications that involve computer vision, it is usually better to disable `autofocus`, and set `focus` to infinity (which is `0` for this node).  Although you lose the larger depth-of-field available when autofocus is enabled, disabling autofocus will ensure that your computer vision algorithims do not register any artifacts arise as a result of the camera changing focus.

For example, an external video device for computer vision should have `autofocus` and `focus` parameters that look something like:

```
<param name="autofocus" value="false" />
<param name="focus" value="0" />
```
