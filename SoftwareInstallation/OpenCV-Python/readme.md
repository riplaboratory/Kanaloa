# OpenCV-Python
OpenCV stands for "Open Computer Vision", and is a collection of C/C++ libraries specific for computer vison.  It has interfaces/APIs in C++, Python, and Java. 

## Prerequisites
1. Ubuntu 16.04 ([instructions here](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/Ubuntu)).
2. Standard .bashrc inclusions ([instructions here](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/.bashrc_inclusions)).
3. ROS Kinetic ([instructions here](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/ROS/Kinetic/InstallatonInstructions)).  Note that ROS is not actually required for this installation, rather ROS causes some issues with the OpenCV Python installation, and therefore some workaround instructions are provided in this guide.
4. Anaconda Python 2.7 and Python 3.6 install ([instructions here](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/AnacondaPython2.7-3.6)).

## Alternative Installation Options
OpenCV-Python can be installed using apt repositories (e.g. `sudo apt-get install python-opencv`); however, these repositories are generally out of date for OpenCV, and therefore are not ideal.  Aternatively, one can build from source; however, the official installation instructions found [at this link](https://docs.opencv.org/3.4.1/d2/de6/tutorial_py_setup_in_ubuntu.html) are fairly poor.  There are third party instrcutions for installing from source like those from [Manuel Ignacio López Quintero](https://milq.github.io/install-opencv-ubuntu-debian/), [pyimagesearch](https://www.pyimagesearch.com/2016/10/24/ubuntu-16-04-how-to-install-opencv/), [Learn Open CV](https://www.learnopencv.com/install-opencv3-on-ubuntu/), and [SciVision](https://www.scivision.co/install-opencv-python-windows/); however, this is still a challenging process non-advanced Linux users due to the many versions of OpenCV, Ubuntu, and associated dependencies.  

In our case, because we already use the excellent Anaconda Python package installer, it makes a lot of sense to install OpenCV this way.  

On another note, ROS Kinetic includes a version of the OpenCV-Python library in its default installation.  However, because it is packaged with ROS, this version is not necessarily updated as often as is desriable.  In these instructions, we will install (and import) the most up-to-date version of the OpenCV-Python library without conflicting with the version installed with ROS.  

## FAQs
__What is the difference between OpenCV 2 and OpenCV 3?__

Based on the name, you may infer than OpenCV 2 is the distribution of OpenCV for Python 2.7, and OpenCV 3 is the distribution of OpenCV for Python 3.x; however, this is not the case.  Simply put, OpenCV 2 is an older version of OpenCV.  OpenCV 3 is the most current version (at the time of writing) with all of the newest library wrappers.  In short, you should be using OpenCV 3.

__If I am using OpenCV 3, why is the Python import call `import cv2`?  Shouldn't it be `import cv3`?__

Admittedly, this is confusing.  [According to this link](https://stackoverflow.com/questions/44102738/why-cant-i-import-opencv3-even-though-the-package-is-installed), although you are using OpenCV 3, the Python library is still called `cv2` because `cv2` is *not* named after the version of OpenCV, it is named after the actual C++ API underneath, which is, to be contrasted from the older C API, named `cv2`.  So the module will likely be called `cv2` for a very long time (regardless of the actual version of OpenCV).  For brevity, it is common to `import cv2 as cv` in Python.  

__Doesn't ROS come with a version of OpenCV?__

Yes.  But because it is distributed with ROS, it is not necessarily the most up-to-date distribution an OpenCV developer would desire.  This tutorial will show you how to safely install, and import the newest version of the OpenCV-Python library without conflicting with the version installed with ROS.  

## Installing OpenCV into your `base` and `py36` virtual environment using Anaconda
If you followed the instructions on the Anaconda Python 2.7 and Python 3.6 install carefully, you should now have an installation of Python 2.7 in your base Python environment, and an installation of Python 3.6 in an environment called `py36`.  Although OpenCV supports both Python 2.7 and 3.6, ROS only supports Python 2.7, which means that we will be using the `base` environment when we need to interact between OpenCV and ROS; however, for sake of completeness, we will install the respective version of the OpenCV Python library to both the `base` (Python 2.7) and `py36` (Python 3.6) environments of our computers.

First, you can check for the latest version of OpenCV in the conda package installer by typing:

```
conda search opencv
```

This will list all of the packages in the `conda` package installer that have the word 'opencv' in its name.  At the time of writing, the latest version of opencv available is version 3.4.1.  To install, type:

```
conda install opencv
```

`conda` should then prompt you to install the latest version of OpenCV it has in it's package repository.  Alternatively, if you wish to explicitly specify a verion of opencv to install, you can type:

```
conda install opencv=3.4.1
```

Accept all of the prompts on the installer screens.  You should now have the Python 2.7 version of OpenCV installed onto your computers `base` environment.  

Now to install OpenCV into our `py36` environment, first enter the `py36` environment by typing:

```
source activate py36
```

Identical to before, to install OpenCV, type:

```
conda install opencv
```

Double check to make sure that the OpenCV builds you are installing are for Python 3.6 and *not* Python 2.7.  As long as you are inside the `py36` environment, `conda` should be smart enough to figure this out.  

You should now have The Python 2.7 OpenCV library installed in your `base` (Python 2.7) environment, and you should have the Python 3.6 OpenCV library installed in your `py36` (Python 3.6) environment.

## Avoiding conflicting OpenCV Python library import calls (and any Python library imports for that matter)
As alluded to in the FAQ section, the default installation of ROS comes with a version of OpenCV.  For sake of reference, for a default ROS Kinetic installation, this OpenCV Python library is located in `/opt/ros/kinetic/lib/python2.7/dist-packages`.  In the last step of this tutorial, we installed the most up-to-date version of OpenCV into our `py27` virtual environment.  For sake of reference, all `conda` virtual environments are stored in `~/anaconda3/envs`, and therefore, our up-to-date OpenCV Python library is stored in `~/anaconda3/envs/py27/lib/python2.7/site-packages`.

Although these Python libraries are safely isolated in different directories, any running version of Python can call them, regardless of the virtual environment its running in, provided that Python knows where the symbolic link (`cv2.so` in our case) is located.  This information is stored in your `PYTHONPATH` variable, which is loaded each time a new Python interpreter is loaded.  In a Unix system, you can control what ends up on the `PYTHONPATH` using the `.bashrc` scipt.  As long as you launch Python (or your Python IDE) from a terminal window sourcing .bashrc, your `PYTHONPATH` will be repeatably generated.

Because both ROS and Anaconda have source inclusions to the `.bashrc` (see our [standard .bashrc inclusions](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/.bashrc_inclusions), all of the Python libraries contained within those installation directories are on the `PYTHONPATH`.  Naturally, this means that the OpenCV 3 library, `cv2`, has symbolic links on the `PYTHONPATH` from both `/opt/ros/kinetic/lib/python2.7/dist-packages` (ROS install), and `~/anaconda3/envs/py27/lib/python2.7/site-packages` (Anaconda install).  These may conflict depending on which version you want to use.  

One potential workaround would be to edit the `.bashrc` sources to our liking, e.g. removing the `source /opt/ros/kinetic/setup.bash` line; however, this can cause unexpected behaviors from other elements of ROS that rely on that `.bashrc` inclusion, which is not an ideal solution for non-advanced users.  A more explicit solution is to directly edit the `PYTHONPATH` variable to include or exclude the Python library paths that have conflicts before we make the import calls on a Python script.  

An example Python code that *excludes* (removes) the ROS Python 2.7 OpenCV library from the `PYTHONPATH` is given as.
   
```
# Put this code BEFORE your import call to remove all ROS Python 2.7 libraries.
import sys
if any('/opt/ros/kinetic/lib/python2.7/dist-packages' in s for s in sys.path):
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
else:
    print('/opt/ros/kinetic/lib/python2.7/dist-packages already removed from sys.path')
 import cv2 as cv
```

Conversely, an example Python code that *includes* the ROS Python 2.7 OpenCV library from the `PYTHONPATH` is given as.

```
# Put this code BEFORE your import call to include all ROS Python 2.7 libraries.
import sys
if not any('/opt/ros/kinetic/lib/python2.7/dist-packages' in s for s in sys.path):
    sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
else:
    print('/opt/ros/kinetic/lib/python2.7/dist-packages already added to sys.path')

```

You can insert this code in between your import calls as necessary to include the libraries that you need in your particular script.  

Optionally, if you are interested in seeing vieing a list of the directories on the PYTHONPATH (the .bashrc will not necessarily tell you this explicitly), you may do this with the following Python code:
   
```
import sys
print(sys.path)
print(' ')
```

## Testing your installation with an example Python import script

Launch our `py27` virtual environment by opening a new terminal and entering:

```
source environment py27
```

Open spyder by opening a new terminal window and entering:
 
```
spyder
```

An example highlighting the need for explicit inclusions and exclusions to the `PYTHONPATH` include `rospy` (a ROS Python 2.7 library), and `cv2` (the newer Anaconda version we installed in this tutorial).  In order to import `rospy`, we need to add a directory that is not on the `PYTHONPATH` by default; and in order to import the Anaconda installation of `cv2`, we need to remove the ROS Python 2.7 libraries from the `PYTHONPATH`.  An example of this is given below:

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

This should print which directory lines were added or removed from the PYTHONPATH prior to the `rospy` and `cv2` imports, and print the current rospy name ('/unnamed'), the current running verion of OpenCV (which should be the newest version that you installed in this tutorial), and the directory where the OpenCV symbolic link is located. 
