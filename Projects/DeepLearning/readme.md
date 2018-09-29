# Deep Learning Project Directory 

#### This Directory contains code for applications that utilize the use of or are to be used with Neural Networks or related tasks

## Anaconda Environments
Most of the projects in this directory are run with Anaconda environments in mind. This ensures that from device to device, the code can be shared and run and the same result can be expected. These environments can be found in the 'environments' folder. Each project folder will contain a readme.md file specifying what environment to use.

#### Installing an Environment
To install an environment, download the .yml file that has the correct environment name and operating system. You can download the specific file by using the wget command in your terminal, cloning the repository, or clicking the 'Raw' button on GitHub so you can right click the text and save it to your computer (be sure to select all files option). In these examples, we will be installing the tfdeeplearning environment for the MAC operating system. The following commands can be run in terminal for Linux and MAC users, and Anaconda Prompt for Windows users. 
```sh
wget https://github.com/riplaboratory/Kanaloa/tree/master/Projects/DeepLearning/environments/mac_tfdl_env.yml
```
Now create the environment from the .yml file
```sh
conda env create -f mac_tfdl_env.yml
```
#### Sourcing the Environment
After installing the environment, you need to ensure that your computer is sourced to that environment. This will ensure you are running the same version of python and python module versions to ensure the same results can be achieved of running the same code on different computers. 
##### For MAC and Linux
```sh
source activate tfdeeplearning
```
##### For Windows
```sh
activate tfdeeplearning
```
You will know if your terminal is properly sourced due to the prefix before your computer name in the terminal. For example:
```sh
(tfdeeplearning) raymond@raymond-aspire:~$
```
Now the scripts you run in the terminal will run according to the environment python version and modules. 

More info on installing Anaconda can also be found [here](https://github.com/riplaboratory/Kanaloa/tree/master/Tutorials/SoftwareInstallation/AnacondaPython2.7-3.6) 

## Jupyter Notebooks
Most of the work for deep learning will be done on jupyter notebooks. This will allow for adequate documentation and easily saving results to be shown to others, rather than just sending and executing large python files which will have to be run in order to see the results. To utilize jupyter notebooks with the Anaconda environments, simpy source the environment as specified above, and then run the command:
```sh
jupyter notebook
```
This will open jupyter in your browser, which you can then navigate through your files to view and edit notebooks (extention .ipynb). These notebooks are comprised of 'cells', either markdown or code. These cells can be run by clicking on it and then hitting _Shift + Enter_ keys,  Code cells that are run will have a number next to it to suggest the order in which the code has run. Code can work from ceel to cell, meaning that if a variable is declared in one cell, and that cell is run, that variable can be called in another cell without having to redefine it. 

## Using Git to share notebooks
Git works with many repository service such as GitHub and Bitbucket. Git is used to format the layout of your updates to your repository. Here are some key commands that can be run via terminal if using the Kanaloa GitHub repository:
Clone (Download) the repository (1 time only)
```sh
git clone https://github.com/riplaboratory/Kanaloa
```
Update the repository (make sure you are in the Kanaloa folder on your terminal using cd, do this command every time before working on repo to get latest updates, you will need permission to the repo to do this)
```sh
git pull
```
Check the status of any changes you have made (this will show you what files you have made changes to that aren't uploaded to the github repository)
```sh
git status
```
Add changes you have made (replace filename with the name of file)
```sh
git add filename    
```
Or to add all changed files
```sh
git add -A
```
Commit the changes (Tell us why you are making these changes)
```sh
git commit -m "Reason why you are uploading update"
```
Actually uploading it to the repo so everyone can use it / see it (you will need permission to the repo to do this, hopefully Brennan has approved you)
```sh
git push
```