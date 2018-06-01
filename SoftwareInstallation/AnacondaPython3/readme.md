# Python 3 install
Python is an open-source programming language.  A Python *distribution* is a Python intepreter packaged with a number of Python libraries for various computing tasks.  We are using the [Anaconda](https://www.anaconda.com/) distribution, which is targeted for data science computing, and is available for both Windows and Linux.  

Note that these instructions are targeted at Python 3.x; however, they may also be used for the legacy Python 2.7 as well.  

## Prerequisites
   1. Ubuntu 16.04 ([instructions here](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/Ubuntu)).
   2. Standard .bashrc inclusions ([instructions here](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/.bashrc_inclusions)).

## Anaconda installation instructions for Linux (Ubuntu)
Official installation instructions for Anaconda can be found [at this link](https://docs.anaconda.com/anaconda/install/linux).

First, download the [Anaconda installer](https://www.anaconda.com/download/#linux) (download the Python 3.x version)
 
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
