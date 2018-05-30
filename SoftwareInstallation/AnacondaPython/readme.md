# Python install
Python is an open-source programming language.  A Python *distribution* is a Python intepreter packaged with a number of Python libraries for various computing tasks.  We are using the [Anaconda](https://www.anaconda.com/) distribution, which is targeted for data science computing, and is available for both Windows and Linux.  

## Prerequisites
   1. Ubuntu 16.04 ([instructions here](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/Ubuntu)).
   2. Standard .bashrc inclusions ([instructions here](https://github.com/riplaboratory/Kanaloa/tree/master/SoftwareInstallation/.bashrc_inclusions)).

## Anaconda installation instructions for Linux (Ubuntu)
Official installation instructions for Anaconda can be found [at this link](https://docs.anaconda.com/anaconda/install/linux).

   1. Download the [Anaconda installer](https://www.anaconda.com/download/#linux)
   2. Navigate to the file you downloaded in terminal using `cd`.  Then run the file using:

```
bash Anaconda-latest-Linux-x86_64.sh
```
   You must change `"latest"` to the version of Anaconda that you downloaded.  Accept all of the prompts on the installer screens.
   
   3. Close and re-open your terminal session.  
   4. Test your installation using

```
conda list
```
   Which should bring up a list of packages.
