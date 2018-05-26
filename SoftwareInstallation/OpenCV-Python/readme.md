# OpenCV-Python
OpenCV stands for "Open Computer Vision", and is a collection of C/C++ libraries specific for computer vison.  It has interfaces/APIs in C++, Python, and Java. 

## Installation instructions
OpenCV-Python can be installed using apt repositoires (e.g. `sudo apt-get install python-opencv`); however, these repositories are generally out of date for OpenCV.  Thusly, building from source is necessary.  The official installation instructions can be found [at this link](https://docs.opencv.org/3.4.1/d2/de6/tutorial_py_setup_in_ubuntu.html); however, these instructions are fairly poor.  Third party instructions from Manuel Ignacio López Quintero, and pyimagesearch.com can be found [at this link](https://milq.github.io/install-opencv-ubuntu-debian/), and [at this link](https://www.pyimagesearch.com/2016/10/24/ubuntu-16-04-how-to-install-opencv/), respectively.

1. If you installed an earlier release of OpenCV using Apt, uninstall using:

```
sudo apt-get autoremove libopencv-dev python-opencv
```

2. Update Apt repositories

```
sudo apt-get update
sudo apt-get upgrade
```

3. Install the dependencies.  Note that there is some flexibility on which packages are necessary here.  This is a generous list of dependencies, many of which are probably already installed on your system:
  -`build-essential` developer tools
  -`pkg-config` developer tools
  -`cmake' developer tools
  -`libjpeg8-dev`, `libtiff5-dev`, `libjasper-dev`, `libpng12-dev` image I/O libraries
  -`libavcodec-dev`, `libavformat-dev`, `libswscale-dev`, `libv4l-dev`, `libxvidcore-dev`, `libx264-dev` video I/O libraries
  -
  

```
sudo apt-get install build-essential cmake pkg-config
sudo apt-get install libjpeg8-dev libtiff5-dev libjasper-dev libpng12-dev
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev
sudo apt-get install -y python-dev python-tk python-numpy python3-dev python3-tk python3-numpy


sudo apt-get install cmake
sudo apt-get install python-devel numpy
sudo apt-get install gcc gcc-c++
```
Type `y` to confirm the install.

2.
