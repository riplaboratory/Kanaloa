# OpenCV-Python
OpenCV stands for "Open Computer Vision", and is a collection of C/C++ libraries specific for computer vison.  It has interfaces/APIs in C++, Python, and Java. 

# This is still under work...

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

conda update -n base conda

   3. Install the dependencies.  Note that there is some flexibility on which packages are necessary here.  This is a generous list of dependencies, some of which may already be installed on your system:

```
sudo apt-get install -y build-essential cmake pkg-config
sudo apt-get install -y libjpeg8-dev libtiff5-dev libjasper-dev libpng12-dev
sudo apt-get install -y libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev
sudo apt-get install -y libgtk-3-dev

# Remove any previous installations of x264
sudo apt-get remove -y x264 libx264-dev

# Necessary dependencies
sudo apt-get install -y build-essential checkinstall cmake pkg-config yasm
sudo apt-get install -y git gfortran
sudo apt-get install -y libjpeg8-dev libjasper-dev libpng12-dev
sudo apt-get install -y libtiff5-dev   # Ubuntu 16.04 only!
sudo apt-get install -y libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev
sudo apt-get install -y libxine2-dev libv4l-dev
sudo apt-get install -y libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
sudo apt-get install -y qt5-default libgtk2.0-dev libtbb-dev
sudo apt-get install -y libatlas-base-dev
sudo apt-get install -y libfaac-dev libmp3lame-dev libtheora-dev
sudo apt-get install -y libvorbis-dev libxvidcore-dev
sudo apt-get install -y libopencore-amrnb-dev libopencore-amrwb-dev
sudo apt-get install -y x264 v4l-utils

# Optional dependencies
sudo apt-get install -y libprotobuf-dev protobuf-compiler
sudo apt-get install -y libgoogle-glog-dev libgflags-dev
sudo apt-get install -y libgphoto2-dev libeigen3-dev libhdf5-dev doxygen

# Python libraries
sudo apt-get install -y python-dev python-tk python-numpy python-pip python3-dev python3-tk python3-numpy python3-pip
sudo apt-get install -y python-testresources python3-testresources
sudo -H pip2 install -U pip numpy
sudo -H pip3 install -U pip numpy

# Install Python virtual environment
sudo -H pip2 install virtualenv virtualenvwrapper
sudo -H pip3 install virtualenv virtualenvwrapper
echo "# Virtual Environment Wrapper inclusions"  >> ~/.bashrc
echo "source /usr/local/bin/virtualenvwrapper.sh" >> ~/.bashrc
source ~/.bashrc
  
# For Python 2
mkvirtualenv facecourse-py2 -p python2
workon facecourse-py2
pip install numpy scipy matplotlib scikit-image scikit-learn ipython
deactivateF

# For Python 3
mkvirtualenv facecourse-py3 -p python3
workon facecourse-py3
pip install numpy scipy matplotlib scikit-image scikit-learn ipython
deactivate



```

   4. Download OpenCV from source.  The easiest way to do ensure you have the most up-to-date version is by cloning their official git:

```
cd ~
sudo apt-get install git
git clone https://github.com/opencv/opencv.git
cd opencv 
git checkout 3.3.1 
cd ~
git clone https://github.com/opencv/opencv_contrib.git
cd opencv_contrib
git checkout 3.3.1
cd ~
```

   5. Setup your Python environment, and virtual Python environments (not necessary but highly recommended).

```
cd ~
wget https://bootstrap.pypa.io/get-pip.py
sudo python get-pip.py
sudo apt-get install -y python3-testresources
sudo pip install virtualenv virtualenvwrapper
sudo rm -rf ~/get-pip.py ~/.cache/pip
```

   6. Add our standard .bashrc inclusions.  Detailed instrcuctions can be found in [this directory](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/.bashrc_inclusions).
   
   7. Update your current terminal session with the changes to `.bashrc`.

```
source ~/.bashrc
```
   Alternatively, you may simply close the current terminal session, and open a new one.
   
   8. Confiure and build using `CMake`.

```
cd ~/opencv/
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
   -D CMAKE_INSTALL_PREFIX=/usr/local \
   -D INSTALL_PYTHON_EXAMPLES=ON \
   -D INSTALL_C_EXAMPLES=OFF \
   -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
   -D PYTHON_EXECUTABLE=~/.virtualenvs/cv/bin/python \
   -D BUILD_EXAMPLES=ON .. 
```

   If the CMake command existed without any errors, you can now compile OpenCV using:

```
make -j4
```

   The `-j4` controls the number of processors to be used in the compilation process.  Sometimes selecting too many processors will cause the compile to bomb out.  If this occurs, then try again using: 

```
make clean
make
```

   9.
   
## Testing your installation

   1. 
   
   
