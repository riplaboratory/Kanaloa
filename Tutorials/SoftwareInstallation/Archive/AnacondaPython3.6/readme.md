# Anaconda Python 3.6 install
Python is an open-source programming language.  A Python *distribution* is a Python intepreter packaged with a number of Python libraries for various computing tasks.  We are using the [Anaconda](https://www.anaconda.com/) distribution, which is targeted for data science computing, and is available for both Windows and Linux.  

Note that these instructions are targeted at Python 3.6.

## Prerequisites
   1. Ubuntu 16.04 ([instructions here](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/Ubuntu)).
   2. Standard .bashrc inclusions ([instructions here](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/.bashrc_inclusions)).

## Anaconda installation instructions for Linux (Ubuntu)
Official installation instructions for Anaconda can be found [at this link](https://docs.anaconda.com/anaconda/install/linux).  Our instructions are a more concise version of the official ones.

First, download the [Anaconda installer](https://www.anaconda.com/download/#linux) (download the Python 3.6 version)
 
Navigate to the file you downloaded in terminal using `cd`.  Assuming this is in your Downloads folder, this would be:

```
cd ~/Downloads
```

Run the downloaded file using:

```
bash Anaconda-latest-Linux-x86_64.sh
```

__Note: you must change `"latest"` to the version of Anaconda that you downloaded__.  Accept all of the prompts on the installer screens.

When this is done, close and re-open your terminal session.  You can then test your installation using

```
conda list
```

Which should bring up a list of packages.  If it does, you are successful!

## Avoiding conflicting Python installations using virtual environments
You may already have installed, or need to install, multiple versions of Python.  This is because Python 2.7 (legacy), is incompatible with the current Python 3.6.  ROS in particular, is only compatible with Python 2.7.  If this is the case, it is no problem to install the Anaconda Python 2.7 distribution and the Anaconda 3.6 distribution concurrently on the same machine; however, you can run into problems when *running* Python (e.g. how do I launch a certain edition of Python, particularly with IDEs?). 

This is a common issue with heavy Python users, and it can be solved by running Python in a virtual environment.  A virtual environment will allow us to run versions of Python that are isolated from the system site directories, with their own Python binaries (aka: any verion of Python), and their own independent set of Python packages.  

There are two supported Python modules for doing this:
   - `venv`: documentation [at this link](https://docs.python.org/3/library/venv.html#module-venv)
   - `conda` environment manager: documentation [at this link](https://conda.io/docs/user-guide/tasks/manage-environments.html)

`venv` is natively supported by Python, whereas the `conda` environment manager is a part of the conda package installer (part of the Anaconda Python distribution).  Both solutions will work for our purposes; however, because we are using the conda package installer [the general advice](https://stackoverflow.com/questions/34398676/does-conda-replace-the-need-for-virtualenv) is to use the `conda` environment manager.

By default, your vitual environments are stored in your Anaconda folder `.../anaconda3/envs/`.  TO create a new environment, open a new terminal and type: 

```
conda create -n py36 python=3.6 anaconda
```

Accept all of the prompts in the terminal window.  

This will create a new environment in Python 3.6.  It is possible to change the Python interpreter version, and instantiate the environment with specific packages.  For more details, read the conda documentation link above.  To enter the new `py36` environment you just created, type:

```
source activate py36
```

You should now be inside the `py36` environment.  You can tell by the `(py36)` that proceeds all of the lines in your terminal.  You can verify that your current environment uses Python 3.6 by typing:

```
python --version
```

You can readout all of your environments (and tell which on you're currently in) by typing:
   
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
