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

3. Install the dependencies.  Note that there is some flexibility on which packages are necessary here.  This is a generous list of dependencies, some of which may already be installed on your system:
   1. `build-essential` developer tools
   2. `pkg-config` developer tools
   3. `cmake` developer tools
   4. `libjpeg8-dev`, `libtiff5-dev`, `libjasper-dev`, `libpng12-dev` image I/O libraries
   5. `libavcodec-dev`, `libavformat-dev`, `libswscale-dev`, `libv4l-dev`, `libxvidcore-dev`, `libx264-dev` video I/O libraries
   6. `libgtk-3-dev` GUI library
   7. `python3` libraries

```
sudo apt-get install -y build-essential cmake pkg-config
sudo apt-get install -y libjpeg8-dev libtiff5-dev libjasper-dev libpng12-dev
sudo apt-get install -y libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev
sudo apt-get install -y libgtk-3-dev
sudo apt-get install -y python-dev python-tk python-numpy python3-dev python3-tk python3-numpy
```

4. Download OpenCV from source.  The easiest way to do ensure you have the most up-to-date version is by cloning their official git:

```
cd ~
sudo apt-get install git
git clone https://github.com/opencv/opencv.git
```

