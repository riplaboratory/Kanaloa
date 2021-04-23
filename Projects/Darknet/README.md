## Kanaloa's Darknet
This documentation is written for the purpose of setting up and configuring Darknet for Kanaloa's participation in RobotX competition. It is being utilized For the purpose of object classification for objects found in the competition.

![](./images/VRX_VORCobjects.png)

### 1. Darknet Installation
Darknet is an open source neural network framework written in C and CUDA by Joseph Redmon ([pjreddie](https://github.com/pjreddie/darknet)). The one that Kanaloa uses is a forked repository that has been modified by [AlexeyAB](https://github.com/AlexeyAB/darknet).

To install this version of Darknet, run
```bash
git clone https://github.com/AlexeyAB/darknet.git
```

Once it your have cloned the repository to your computer, change directories to the Darknet workspace.
```sh
cd darknet
```
Then compile
```sh
make
```
NOTE: If you have a GPU, CUDNN, and OpenCV installed, change parameters `GPU`, `CUDNN`, and `OPENCV` to 1 so that training will go much faster. NOTE: Be sure not to leave any whitespaces as it can cause compiling errors when running `make`. By default, you will see this at the top when editing `Makefile`
```Makefile
GPU=0
CUDNN=0
CUDNN_HALF=0
OPENCV=0
...
```

### 2. Darknet Set-up
Note: File names used in this documentation are just examples. It is highly suggest you used detailed names as there are many `.cfg` and `.data` files that come included with the cloned darknet repository.

#### 2.1 Darknet Configuration Files
Darknet requires the usage of configurations files (`.cfg`) for training. 
It contains parameters such as:
- learning_rate
- angle
- saturation
- exposure
- hue

Structure of the neural network layers:
- filters
- activation
- stride

Then the You Only Look Once (YOLO) layers and number of filters that will need to be modified:
- classes = # of classes in dataset
- filters = (classes + coordinates + 1) * mask
    - coordinates will be 4 as we have: center x, center y, width, and height, from our YOLO files

#### 2.2 Darknet Needed Files
Darknet requires the following files:
- `darknet/images/classes.names`
- `darknet/cfg/custom_data.data`
- `darknet/cfg/custom_train.cfg` (name to match network/usage)
- `darknet/cfg/custom_test.cfg` (optional, name to match network/usage)
- `darknet/images/train.txt`
- `darknet/images/test.txt`
- `darknet/weights` can be found [here](https://drive.google.com/drive/folders/1avzyAFq82Vc5SBlOv1QPWNvzBRWjOVTQ)

##### 2.2.1 Classes
The `classes.names` file should include the names of the classes you will be training Darknet to look for. These names should be in the same order as the classes defined in your YOLO files. If your YOLO classes are predefined in this order
```
surmark
totem
polyform
```
Then your `classes.names` file will be written in the same exact way. 

##### 2.2.2 Custom Data
`custom_data.data` should include the following information:
- number of classes
- path to `train.txt`
- path to `test.txt`
- path to `classes.name`
- backup name

Here is an example of how it may look in the end
```
classes = 3
train = /home/kevin/Desktop/darknet/images/train.txt
valid = /home/kevin/Desktop/darknet/images/test.txt
names = /home/kevin/Desktop/darknet/images/classes.names
backup = backup
```

##### 2.2.3 Custom Train and Test
These two files are configured slightly different. You can find the full files Kanaloa uses [here](). 

#### 2.3 Configuring Makefile
By default, you will see this at the top when editing `Makefile`
```Makefile
GPU=0
CUDNN=0
CUDNN_HALF=0
OPENCV=0
...
```
If you have a GPU, CUDNN, and OpenCV installed, change parameters `GPU`, `CUDNN`, and `OPENCV` to 1 so that training will go much faster. NOTE: Be sure not to leave any whitespaces as it can cause compiling errors when running `make`.

### 3. Usage
The following shows how to use Darknet to train and test the network.

#### 3.1 Train Network

```
./darknet detector train cfg/custom_data.data cfg/custom_data.cfg weights/darknet53.conv.74 -dont_show
```

In this example, weights/darknet53.conv.74 is previously trained weights from a similar network to implement transfer learning.

#### 3.2 Test Network

```
./darknet detector test cfg/custom_data.data cfg/custom_data.cfg backup/yolov3_kanaloa_oc_train_2000.weights data/image.jpg
```

### 4. Extra Notes
- If you are looking to use Darknet for live obect classification through a video feed, you will need to install:
    - CUDA >= 10.0
    - OpenCV >= 2.4
    - cuDNN >= 7.0
    - GPU with CC >= 3.0
- AlexeyAB's version of Darknet is compatible with a Windows machine.
    - Team Kanaloa uses the Linux based version since ROS is being used and live data will be taken from there
    - For training and testing image labeling, the weights will be the same across both operating systems