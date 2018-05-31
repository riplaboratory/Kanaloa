# OpenCV-Python
OpenCV stands for "Open Computer Vision", and is a collection of C/C++ libraries specific for computer vison.  It has interfaces/APIs in C++, Python, and Java. 

## Alternative Installation Options
OpenCV-Python can be insatlled using apt repositories (e.g. `sudo apt-get install python-opencv`); however, these repositories are generally out of date for OpenCV, and therefore are not ideal.  Aternatively, one can build from source; however, the official installation instructions found [at this link](https://docs.opencv.org/3.4.1/d2/de6/tutorial_py_setup_in_ubuntu.html) are fairly poor.  There are third party instrcutions for installing from source like those from [Manuel Ignacio López Quintero](https://milq.github.io/install-opencv-ubuntu-debian/), [pyimagesearch](https://www.pyimagesearch.com/2016/10/24/ubuntu-16-04-how-to-install-opencv/), [Learn Open CV](https://www.learnopencv.com/install-opencv3-on-ubuntu/), and [SciVision](https://www.scivision.co/install-opencv-python-windows/); however, this is still a challenging process non-advanced Linux users due to the many versions of OpenCV, Ubuntu, and associated dependencies.  

In our case, because we already use the excellent Anaconda Python package installer, it makes a lot of sense to install OpenCV this way.  

## Prerequisites
   1. Ubuntu 16.04 ([instructions here](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/Ubuntu)).
   2. Anaconda Python installation ([instructions here](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/AnacondaPython)).
   3. ROS Kinetic ([instructions here](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/ROS/Kinetic/InstallatonInstructions)).  Note that ROS is not actually required for this installation, rather ROS causes some issues with the OpenCV Python installation, and therefore some workaround instructions are provided in this guide.
   4. Standard .bashrc inclusions ([instructions here](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/.bashrc_inclusions)).

## Installation instructions

   1. If you installed an earlier release of OpenCV using Apt, uninstall using:

```
sudo apt-get autoremove libopencv-dev python-opencv
```

   2. Update Apt repositories

```
sudo apt-get update
sudo apt-get upgrade
```

   3. Install OpenCV and OpenCV3 using Anaconda.  Note that we are getting OpenCV through [conda-forge](https://anaconda.org/conda-forge/opencv), and OpenCV3 through [menpo](https://anaconda.org/menpo/opencv3).

```
conda update -n base conda
conda install -c conda-forge opencv 
conda install -c menpo opencv3
```


## Addressing conflicting Python installaions using virtual environments
Because ROS Kinetic (or any edition of ROS for that matter)  natively uses Python 2.7, and *additionally* has it's own built-in computer vision file `cv2.so` (which natively overrides any other Python import request to OpenCV), we're in a situation where we have conflicting editions of Python (a common problem for heavy Python users).  We can address this problem by running Python in a virtual environment.  This will allow us to run versions of Python that are isolated from the system site directories, with their own Python binaries (aka: any verion of Python), and their own independent set of Python packages.  

There are two supported Python modules for doing this:
   - `venv`: documentation [at this link](https://docs.python.org/3/library/venv.html#module-venv)
   - `conda create`: documentation [at this link](https://conda.io/docs/user-guide/tasks/manage-environments.html)

`venv` is natively supported by Python, whereas `conda create` is a part of the conda package installer (part of the Anaconda Python distribution).  Both sotluions will work; because we are using the conda package installer, [the general advice](https://stackoverflow.com/questions/34398676/does-conda-replace-the-need-for-virtualenv) is to use the `conda create` solution.

   By default, your vitual environments are stored in your Anaconda folder `.../anaconda3/envs/`.  TO create a new environment, type: 

```
conda create --name ocv2 
```

   This will create a new environment in the current running version of Python (3.x).  It is possible to change the Python interpreter version, and instantiate the environment with specific packages.  For more details, read the conda documentation link above.  

   To enter the new `ocv2` environment you just created, type:

```
source activate ocv2
```

   You should now be inside the `ocv2` environment.  You can tell by the `(ocv2)` that proceeds all of the lines in your terminal.  You can readout all of your environments (and tell which on you're currently in) by typing:
   
```
conda info -e
```

   You can start writing Python code in this new environment directly in the terminal.  You can also run an IDE inside this new environment.  To launch the Spyder IDE, type:
   
```
spyder
```
   
   And Spyder should launch inside this new environment.  
   
   In order to quit out of the environment, type:
   
```
source deactivate
```

## Setting up ocv2 conda virtual environment
At this point, we installed Anaconda Python 3.x distribution with the conda package installer, and set up a Python 3.x virtual environment called `ocv2` to prevent us from conflicting with 


__on simply removing ROS Python library from PYTHONPATH__:
This is located in the `/opt/ros/kinetic/lib/python2.7/dist-packages`, which is called when ROS is activated in our .bashrc inclusions (`source /opt/ros/kinetic/setup.bash`).  There are a few workarounds for this; we could remove that line from the .bashrc inclusions; however, that will mean that any Python 2.7 script called from ther terminal in ROS will break, which is not an ideal solution.  Instead, ensure that any Python script you write that utilizes OpenCV contains the folloing code before the import calls:
   
```
while True:
    try:
        sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
        break
    except ValueError:
        print('/opt/ros/kinetic/lib/python2.7/dist-packages aready removed from sys.path')
        print(' ')
        break
```
   This code will remove the `/opt/ros/kinetic/lib/python2.7/dist-packages` from the PYTHONPATH, leaving only the Anaconda (and any other import paths you may have on your system) import path.  The try-catch syntax is necessary because after the path has been removed from the interpreter, subsequent calls will throw an error (trying to remove a path that does not exist).  


## Testing your installation
__this section is under development__

   1. Open spyder by opening a new terminal window and entering:
 
```
spyder
```

   2. Create a new script, and enter the following:

```
import sys
print(sys.path)
print(' ')

while True:
    try:
        sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
        print('removing /opt/ros/kinetic/lib/python2.7/dist-packages from sys.path')
        print(' ')
        break
    except ValueError:
        print('/opt/ros/kinetic/lib/python2.7/dist-packages aready removed from sys.path')
        print(' ')
        break

import cv2 as cv
print(' ')
print(cv.__version__)
print(' ')
```

   This should return the list of directories in the current PYTHONPATH, and the version of opencv.
   
   3. Run it a second time, this time, the list of directories on the current PYTHONPATH should no longer contain `'/opt/ros/kinetic/lib/python2.7/dist-packages'`.
   

## Extra stuff (under development)

Dependencies: Note that there is some flexibility on which packages are necessary here.  This is a generous list of dependencies, some of which may already be installed on your system:

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
