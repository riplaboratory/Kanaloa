# Anaconda Python 2.7 and Python 3.6 install
Python is an open-source programming language.  A Python *distribution* is a Python intepreter packaged with a number of Python libraries for various computing tasks.  We are using the [Anaconda](https://www.anaconda.com/) distribution, which is targeted for data science computing, and is available for both Windows and Linux.  

In this tutorial, we will install BOTH Python 2.7 and Python 3.6, and instruct you how to manage both installations.  

## Prerequisites
   1. Ubuntu 16.04 ([instructions here](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/Ubuntu)).
   2. Standard .bashrc inclusions ([instructions here](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/.bashrc_inclusions)).
   
## Python 2.7 vs 3.6
At the time of writing, the most up-to-date version of Python is Python 3, particularly Python 3.6.  Python 3 is *not* backwards compatible with the legacy Python 2 (the last version of which is Python 2.7); however, because there are still a lot of Python applications that have not been updated to support Python 3, and therefore, will only run in a Python 2.7 environment.  In our case, ROS only supports Python 2.7, while OpenCV supports both Python 2.7 and Python 3.6.  For this reason, there is a lot of value in having working, non-conflicting binaries for both Python 2.7 and Python 3.6.

In this tutorial, we will install Python 2.7 using the Anaconda 2 distribution, *and then* install Python 3.6 in a virtual environment called `py36`.  This will allow us to write software for ROS primarily in Python 2.7, while still having a safely-isolated installation of Python 3.6 that we can access at any time.  

## Anaconda 2 installation instructions for Linux (Ubuntu)
Official installation instructions for Anaconda can be found [at this link](https://docs.anaconda.com/anaconda/install/linux).  Our instructions are a more concise version of the official ones.

First, download the [Anaconda installer](https://www.anaconda.com/download/#linux) (download the Python 2.7 version)
 
Navigate to the file you downloaded in terminal using `cd`.  Assuming this is in your Downloads folder, this would be:

```
cd ~/Downloads
```

Run the downloaded file using:

```
bash Anaconda2-latest-Linux-x86_64.sh
```

__Note: you must change `"latest"` to the version of Anaconda that you downloaded__.  Accept all of the prompts on the installer screens.

When this is done, source your .bashrc file to the current terminal window by typing:

```
source .bashrc
```
Alternatively, the .bashrc is reloaded each time a new terminal window is open, so you can simply close and re-open your terminal window.  You can then test your installation by typing:

```
conda list
```

Which should bring up a list of packages.  If it does, you are successful with the Python 2.7 install.

## Avoiding conflicting Python installations using virtual environments
At this point, we only have the Anaconda Python 2.7 distribution installed.  As previously discussed, Python 2.7 is the legacy version of Python, which is now being phased out in favor of the newer Python 3.6.  For this reason, there is a lot of value in having working, non-conflicting binaries for both Python 2.7 and Python 3.6.  This is a common issue with heavy Python users, and it can be accomplished by using Python virtual environments.  Virtual environments allow the different versions of Python to stay isolated from the system site directories, with their own Python binaries and their own independent set of Python packages.  

There are two supported Python modules for doing this:
   - `venv`: documentation [at this link](https://docs.python.org/3/library/venv.html#module-venv)
   - `conda` environment manager: documentation [at this link](https://conda.io/docs/user-guide/tasks/manage-environments.html)

`venv` is natively supported by Python, whereas the `conda` environment manager is a part of the conda package installer (part of the Anaconda Python distribution).  Both solutions will work for our purposes; however, because we are using the conda package installer [the general advice](https://stackoverflow.com/questions/34398676/does-conda-replace-the-need-for-virtualenv) is to use the `conda` environment manager.

Because ROS only supports Python 2.7 for the forseeable future (and OpenCV supports both Python 2.7 and Python 3.6); we have elected to install Python 2.7 on the "base" environment of the computer, and then install Python 3.6 in it's own environment, called `py36`.  We *could* go the other way around (i.e. install Python 3.6 in the base environment, and install Python 2.7 in a virtual environment); however, because the Python 2.7 environment needs to access the Python packages installed by ROS, *and* the packages installed by Anaconda (both of which are stored in different directory trees), this is much easier accomplished if Python 2.7 is in the "base" envrionment.  

By default, your vitual environments are stored in your Anaconda folder `.../anaconda3/envs/`.  You can view all of your conda virtual environents by typing:

```
conda info -e
```

At the moment, we only have a single `base` envrionment.  You can check which version of Python you're running by typing:

```
python --version
```

To create a new environment called `py27` *and* install Python 2.7 onto this environment, open a new terminal and type: 

```
conda create -n py27 python=2.7 anaconda
```

The `conda` package manager should automatically identify all of the packages in their Anaconda Python 2.7 distribution.  Accept all of the prompts in the terminal window to install.  

To enter the new `py27` environment you just created, type:

```
source activate py27
```

You should now be inside the `py27` environment.  You can tell by the `(py27)` that proceeds all of the lines in your terminal.  You can verify that your current environment uses Python 2.7 by typing:

```
python --version
```

Just like in the base environment, you can also view all of your installed packages by typing:
   
```
conda list
```

In this environment, you should should see that these packages are are py27 (Python 2.7) versions, as opposed to the py36 (Python 3.6) versions in the `base` environment.  

You can start writing Python code in this new environment directly in the terminal.  You can also run an IDE inside this new environment.  To launch the Spyder IDE, type:
   
```
spyder
```

And Spyder should launch inside this new environment.  
   
In order to quit out of the environment, type:
   
```
source deactivate
```

## Potential issues
Sypder launches to a black screen with terminal error `shader program is not linked`.  Solution documented [at this link](https://github.com/spyder-ide/spyder/issues/3226#issuecomment-394060919).
