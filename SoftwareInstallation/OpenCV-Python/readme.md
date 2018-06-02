# OpenCV-Python
OpenCV stands for "Open Computer Vision", and is a collection of C/C++ libraries specific for computer vison.  It has interfaces/APIs in C++, Python, and Java. 

## Prerequisites
1. Ubuntu 16.04 ([instructions here](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/Ubuntu)).
2. Standard .bashrc inclusions ([instructions here](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/.bashrc_inclusions)).
3. ROS Kinetic ([instructions here](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/ROS/Kinetic/InstallatonInstructions)).  Note that ROS is not actually required for this installation, rather ROS causes some issues with the OpenCV Python installation, and therefore some workaround instructions are provided in this guide.
4. Anaconda Python 2.7 and Python 3.6 install ([instructions here](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/AnacondaPython2.7-3.6)).

## Alternative Installation Options
OpenCV-Python can be insatlled using apt repositories (e.g. `sudo apt-get install python-opencv`); however, these repositories are generally out of date for OpenCV, and therefore are not ideal.  Aternatively, one can build from source; however, the official installation instructions found [at this link](https://docs.opencv.org/3.4.1/d2/de6/tutorial_py_setup_in_ubuntu.html) are fairly poor.  There are third party instrcutions for installing from source like those from [Manuel Ignacio López Quintero](https://milq.github.io/install-opencv-ubuntu-debian/), [pyimagesearch](https://www.pyimagesearch.com/2016/10/24/ubuntu-16-04-how-to-install-opencv/), [Learn Open CV](https://www.learnopencv.com/install-opencv3-on-ubuntu/), and [SciVision](https://www.scivision.co/install-opencv-python-windows/); however, this is still a challenging process non-advanced Linux users due to the many versions of OpenCV, Ubuntu, and associated dependencies.  

In our case, because we already use the excellent Anaconda Python package installer, it makes a lot of sense to install OpenCV this way.  

On another note, as of writing, ROS Kinetic includes a version of the OpenCV library in its default installation.  However, because it is packaged with ROS, this version is not necessarily updated as often as is desriable.  Following these instructions will 

## FAQs
__What is the difference between OpenCV2 and OpenCV3?__
Based on the name, someone may infer than OpenCV 2 is the distribution of OpenCV for Python 2.7, and OpenCV 3 is the distribution for Python 3.6; however, this is not the case.  Simply put, OpenCV 2 is an older version of OpenCV.  OpenCV 3 is the newest version with all of the newest library wrappers; you should be using OpenCV 3.

__If I am using OpenCV 3, why is the Python import call still `import cv2`?  Shouldn't it be `import cv3`?__
Admittedly, this is confusing.  [According to this link](https://stackoverflow.com/questions/44102738/why-cant-i-import-opencv3-even-though-the-package-is-installed) Although you are using OpenCV 3, the module is still called `cv2` because it doesn't represent the version of OpenCV but the actual C++ API underneath, which is, to be contrasted from the C API, named - `cv2`.  So it will likely be called `cv2` for a very long time.  

__Doesn't ROS come with a version of OpenCV?__
Yes.  But because it is distributed with ROS, it is not necessarily the most up-to-date distribution an OpenCV developer would desire.  This tutorial will show you how to safely manage the newest version of OpenCV alongside the version that already comes with ROS.  

## Setting up ocv2 conda virtual environment
If you followed the instructions on the Anaconda Python 2.7 and Python 3.6 install carefully, you should now have an installation of Python 3.6 in your base Python environment, and an installation of Python 2.7 in an environment called `py27`.  Althought OpenCV supports both Python 2.7 and 3.6, ROS (particularly `rospy`) only supports Python 2.7, which means that we will be using the `py27` environment to interact between OpenCV and ROS.  

Enter the `py27` environment by typing:

```
source activate py27
```

You can check for the latest version of OpenCV in the conda package installer by typing:

```
conda search opencv
```

At the time of writing, the latest version of opencv available is version 3.4.1.  Therefore, to install, type:

```
conda install opencv=3.4.1
```

Conda will then install the version of opencv that you specify.  It should also automatically select the version that matches the Python binary of the environment that you're currently in (Python 2.7 in our case).  You can (and should) double check all of this information before you accept all of the prompts. 



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

With this, you have:
1. Set up a virtual Python environment through the `conda` environment manager.
2. Installed OpenCV and OpenCV3 onto this virtul Python environment through the `conda` package installer.
3. Learned how to manipulate the directories on the PYTHONPATH, to avoid conflicting symbolic import links to Python libraries, particularly `cv2` and `rospy`.
