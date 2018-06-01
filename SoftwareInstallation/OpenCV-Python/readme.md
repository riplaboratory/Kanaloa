# OpenCV-Python
OpenCV stands for "Open Computer Vision", and is a collection of C/C++ libraries specific for computer vison.  It has interfaces/APIs in C++, Python, and Java. 

## Alternative Installation Options
OpenCV-Python can be insatlled using apt repositories (e.g. `sudo apt-get install python-opencv`); however, these repositories are generally out of date for OpenCV, and therefore are not ideal.  Aternatively, one can build from source; however, the official installation instructions found [at this link](https://docs.opencv.org/3.4.1/d2/de6/tutorial_py_setup_in_ubuntu.html) are fairly poor.  There are third party instrcutions for installing from source like those from [Manuel Ignacio López Quintero](https://milq.github.io/install-opencv-ubuntu-debian/), [pyimagesearch](https://www.pyimagesearch.com/2016/10/24/ubuntu-16-04-how-to-install-opencv/), [Learn Open CV](https://www.learnopencv.com/install-opencv3-on-ubuntu/), and [SciVision](https://www.scivision.co/install-opencv-python-windows/); however, this is still a challenging process non-advanced Linux users due to the many versions of OpenCV, Ubuntu, and associated dependencies.  

In our case, because we already use the excellent Anaconda Python package installer, it makes a lot of sense to install OpenCV this way.  

## Prerequisites
1. Ubuntu 16.04 ([instructions here](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/Ubuntu)).
2. Anaconda Python 3.x installation ([instructions here](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/AnacondaPython)).
3. ROS Kinetic ([instructions here](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/ROS/Kinetic/InstallatonInstructions)).  Note that ROS is not actually required for this installation, rather ROS causes some issues with the OpenCV Python installation, and therefore some workaround instructions are provided in this guide.
4. Standard .bashrc inclusions ([instructions here](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/.bashrc_inclusions)).

## Avoiding conflicting Python installations using virtual environments
Because all current versions of ROS natively use Python 2.7, and *additionally* has it's own built-in computer vision library `cv2.so`, we're in a situation where we have conflicting editions of Python *and* conflicting versions of Python libraries.  This is actually a common issue for heavy Python users; and it can be solved by running Python in a virtual environment.  A virtual environment will allow us to run versions of Python that are isolated from the system site directories, with their own Python binaries (aka: any verion of Python), and their own independent set of Python packages.  

There are two supported Python modules for doing this:
   - `venv`: documentation [at this link](https://docs.python.org/3/library/venv.html#module-venv)
   - `conda` environment manager: documentation [at this link](https://conda.io/docs/user-guide/tasks/manage-environments.html)

`venv` is natively supported by Python, whereas the `conda` environment manager is a part of the conda package installer (part of the Anaconda Python distribution).  Both solutions will work for our purposes; however, because we are using the conda package installer [the general advice](https://stackoverflow.com/questions/34398676/does-conda-replace-the-need-for-virtualenv) is to use the `conda` environment manager.

By default, your vitual environments are stored in your Anaconda folder `.../anaconda3/envs/`.  TO create a new environment, open a new terminal and type: 

```
conda create --name ocv2 
```

This will create a new environment in the current running version of Python (3.x).  It is possible to change the Python interpreter version, and instantiate the environment with specific packages.  For more details, read the conda documentation link above.  To enter the new `ocv2` environment you just created, type:

```
source activate ocv2
```

You should now be inside the `ocv2` environment.  You can tell by the `(ocv2)` that proceeds all of the lines in your terminal.  You can readout all of your environments (and tell which on you're currently in) by typing:
   
```
conda info -e
```

Just like in the base environment, you can also view all of your installed packages by typing:
   
```
conda list
```

In this newly created environment, you should see a blank list of packages.

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
At this point, we installed Anaconda Python 3.x distribution with the conda package installer, and set up a Python 3.x virtual environment called `ocv2` to prevent us from conflicting with the Python 2.7 installation already on the machine from the ROS install.  Now, we want to set up this `ocv2` environment with everything relevant to OpenCV-Python.

Enter the `ocv2` environment by typing:

```
source activate ocv2
```

Install `scipy` (a common distribution of Python libraries targeted at mathematical operations).

```
conda install scipy
```

Optionally, you may check that the installtion of the Scipy libraries was successful by typing `conda list`.

Install OpenCV and OpenCV3 using the conda package installer  Note that we are getting OpenCV through [conda-forge](https://anaconda.org/conda-forge/opencv), and OpenCV3 through [menpo](https://anaconda.org/menpo/opencv3).

```
conda install -c conda-forge opencv 
conda install -c menpo opencv3
```
   
Similar to before, you may optionally check that the installtion of the OpenCV libraries was successful by typing `conda list`.

## Avoiding conflicting Python libraries
We now have Python 3.x virtual environment called `ocv2` running the `opencv` and `opencv3` packages.  Although this virutal environment is completely separate from any other Python environments on your system, it was (presumably) launched through a terminal session with our standard `.bashrc` inclusions.  This means that the ROS Python 2.7 `dist-packages` (located in the `/opt/ros/kinetic/lib/python2.7/dist-packages` directory in a default ROS Kinetic installation) directory was still included in the `PYTHONPATH` (the list of symbolic links Python looks through to find packages), and therefore can (and does) still conflict with OpenCV (and potentially other packages) import calls in our `ocv2` environment.  For reference, the ROS Python 2.7 symbolic link is located in `/opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so`, and the Anaconda Python 3.6 symbolic link is located in `/home/USERNAME/anaconda3/lib/python3.6/site-packages/cv2.cpython-36m-x86_64-linux-gnu.so` (both for default installations).  

One potential workaround would be to remove that line `source /opt/ros/kinetic/setup.bash` from the .bashrc inclusions.  This will prevent the Python 2.7 packages from ROS from making it onto the `PYTHONPATH`; however, that will mean that any ROS Python 2.7 script called from the terminal will break, which is not an ideal solution.  A more explicit (but less universal) solution would be to explicitly remove the `/opt/ros/kinetic/lib/python2.7/dist-packages` from the PYTHONPATH at the start of any python script requiring `cv2` for Python 3.  An error-checking version of this Python code is given below:
   
```
# Put this code BEFORE your cv2 import call to prevent conflicts with the ROS Python 2.7 OpenCV package.
if any('/opt/ros/kinetic/lib/python2.7/dist-packages' in s for s in sys.path):
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
else:
    print('/opt/ros/kinetic/lib/python2.7/dist-packages already removed from sys.path')
```

This code will remove the `/opt/ros/kinetic/lib/python2.7/dist-packages` from the PYTHONPATH, leaving only the Anaconda (and any other import paths you may have on your system) import path.  The path only should be removed once, for a given Python interpreter instantiation; the try-catch will prevent errors for this reason.
   
Optionally, if you are interested in seeing vieing a list of the directories on the PYTHONPATH (the .bashrc will not necessarily tell you this explicitly), you may do this with the following Python code:
   
```
import sys
print(sys.path)
print(' ')
```

## Testing your installation with an example Python import script

Launch our `ocv2` virtual environment by opening a new terminal and entering:

```
source environment ocv2
```

Open spyder by opening a new terminal window and entering:
 
```
spyder
```

A common example of these library import calls together include `rospy` (the ROS Python 2.7 module), and `cv2`.  In order to import `rospy`, we need to add a directory not on the default PYTHONPATH, and in order to import `cv2`, we need to remove all of the ROS Python 2.7 libraries from the PYTHONPATH to avoid the conflicts (as discussed earlier).  An example of this is given below:

```
import sys

# Add rospy library directories to the PYTHONPATH
if not any('/opt/ros/kinetic/lib/python2.7/dist-packages' in s for s in sys.path):
    print('adding /opt/ros/kinetic/lib/python2.7/dist-packages to the PYTHONPATH...')
    sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
if not any('/usr/lib/python2.7/dist-packages' in s for s in sys.path):
    print('adding /usr/lib/python2.7/dist-packages to the PYTHONPATH...')
    sys.path.append('/usr/lib/python2.7/dist-packages')
print('importing rospy...')
import rospy

# Remove ROS Python 2.7 libraries from PYTHONPATH to avoid conflict
if any('/opt/ros/kinetic/lib/python2.7/dist-packages' in s for s in sys.path):
    print('removing /opt/ros/kinetic/lib/python2.7/dist-packages to the PYTHONPATH...')
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
if any('/usr/lib/python2.7/dist-packages' in s for s in sys.path):
    print('removing /usr/lib/python2.7/dist-packages to the PYTHONPATH...')
    sys.path.remove('/usr/lib/python2.7/dist-packages')
print('importing cv2...')
import cv2 as cv

print(' ')
print('rospy name:       ' + rospy.get_name())
print('OpenCV version:   ' + cv.__version__)
print('OpenCV directory: ' + cv.__file__)
```

This should print which directory lines were added or removed from the PYTHONPATH, and print the current rospy name (probably '/unnamed', the current running verion of OpenCV, and the directory where the OpenCV symbolic link is located.   

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
