# Deep Learning with MATLAB for shape recognition

For the Kanaloa team project, we were presented with the task of image detecting shapes in real time for the Maritime Robotx competition for various reasons. These reasons include the detection for docking the boat, detecting waypoints and much more. Due to this, we decided to use some type of Neural Network to solve this issue. Our two, almost defacto choices to do this was MATLAB (R2018b to be exact) and TensorFlow with a custom made Neural Network. We chose these two out of the many other deep learning programs because either their customizability (in TensorFlow’s case) or their (sort of) ease of use (like in MATLAB’s case).

Due to the fact that we tried TensorFlow and its results were clunky and hard to use, we decided to go with MATLAB due to its proven track record with embedded systems (sort of like the NUC in this case) and having a working Convolutional Neural Network Toolkit for download for about 4 years already.

# Design Process:
## Prerequisites:
To start, you need to have MATLAB R2018b installed with these software packages/plugins:

![image](https://github.com/k16blfurm/Kanaloa/blob/master/Projects/Shape%20Recognition/pasted%20image%200.png)

Also, to get the USB Webcam working in in MATLAB, you need the Webcam hardware package add-on from MATLAB.
https://www.mathworks.com/help/supportpkg/usbwebcams/ug/acquire-images-from-webcams.html

## Image Labeling:
After that, you can start collecting data, labeling, and training.

First, we need to collect data from the USB camera by using using the built in camera modules in Windows or Linux.

When we take pictures, they are taken in 1920x1080, which needs to be downscaled to a resolution of 960x540 in order to be labelled, trained, and processed by our scripts.
If the images are taken with a camera that is not the logitech USB webcams, the resolution of the camera must be noted and a new image resize script must be made to crop and downscale the images to the correct resolution.  If the images need to be cropped, they must be cropped in order to prevent image distortion when downscaling.  You can search for the correct resize script in the imageResizeScripts folder in the drive, or copy and modify one to resize images from a completely new resolution.  In this case, we would use bulkResize.m script to downscale the 1920x1080 images.  We can resize them using the imresize or imcrop.

For more information on how imcrop works:
https://www.mathworks.com/help/images/ref/imcrop.html
For more information on how imresize works:
https://www.mathworks.com/help/images/ref/imresize.html

We need to collect as much image data as possible in order to maximize the accuracy of the network model, which means that thousands of images must be obtained.  These images must be taken with different backgrounds in different configurations, lighting conditions, depths, and angles. (In our case, we managed to come up with a little over 3000 images from Campus, Coconut Island, and a couple more. 

So in this certain case, we were feeding in a 16:9 file and not cropping it at all, 

After you convert your images, you need to go to the Apps tab in Matlab and open the Image Labeler app to begin image labelling or add to our existing image labelling session.

Here is an image from our labelling session.
On the left, you can add your labels and you can bounding box the shapes from here by clicking and dragging a box around the shape of interest.  Make sure to label the shapes correctly. 

Once your images have been labelled, you need to export your labels to your workspace for training.  This will create a groundtruth.m file (or whatever you want, making sure to follow the guides set on the github for file naming) which you will need to save in order to utilize the labelled images with the TrainRCNN script.  This script will train the network.

## Training:

Going from here, we can open the TrainRCNN program to train the neural network.  Note that the training will take a while, especially if you are training a few thousand images.  Using a computer with a GPU is highly recommended to speed up the training process.

Here is a screenshot of our code:


This is the section of the code where a training dataset is created from the groundtruth file.  You can also see the image properties.
As we can see above, we can load our labels into the TrainRCNN by using the line of code: groundTruth = load(....).

Also, you can see on the bottom of the image that we can change the Image Properties for what images we are going to send into the network for training.  In our case, we will be using an image resolution of 960x540.

Also, you need to declare your labels “{'circle','cruciform','triangle'}”

From here we can just run the script in MATLAB after this and it will produce a trained RCNN model which we will use to validate the network with. Make sure to save your workspace to save the RCNN since we will be using this later on for validation.

Now, we need to open the TrainingWork.m script to validate our images and test out the network. We first load the RCNN network using this command:

We will load in the data with the load command.


## Validation:

Going from here, we can use our code to change our height and width, as shown below.

Also, we can load our images by using the imread command or load images from the webcam by referencing the webcam in our code (as shown below).

Here is the line of code which processes and classifies an image one at a time.  To use this, you would need to load in all of your validation images into your working directory and type in the name of the image you want to test out within the parenthesis in the imread function.  All you would need to do is replace the purple text with the image name.  Then you would run the script.

Here is the chunk of code that allows the script to work with a live stream of images where the camera automatically takes a picture after every few seconds, resizes the image to the correct resolution, and then classifies the shapes that it sees in the image.  In order to use this portion of the code, you must have a USB webcam plugged into the computer and determine the webcam number called in this line of code:

mycam = webcam(camera number);

You can determine the webcam number by typing this into the command line:

camList = webcamlist

This will print out the camera and camera number that Matlab is seeing.  Usually the Logitech USB webcams will be camera number 2.  The snapshot line in the code is what takes the pictures from the live video stream.  To check if the camera is linked to Matlab and the script, you can type this into the command line:

preview(cam);

This will cause a window to pop up which will display the video that the camera is seeing.  If you are getting any errors while running this script, check to see if Matlab is recognizing the USB webcam and that you have the webcam hardware plugin installed in Matlab.

Here is a link to the documentation for processing images from live video in Matlab:
https://www.mathworks.com/help/supportpkg/usbwebcams/examples/acquiring-a-single-image-in-a-loop.html?prodcode=ML

If you are using a GPU and are getting an error while running the script, this may be caused by the GPU running out of memory.  An example of this error message is shown below:



This is what you’ll get if you try to run a lackluster GPU without enough memory to validate the network. For this step, it’s recommended to run with a CPU if your graphics buffer is less than 4gb (or 3.5gb in my case for the GTX 970).

We can also change the threshold of validation as shown below.  Only bounding boxes that meet or exceed the threshold will be displayed to the screen during validation.  If you run the classifier model and the image is being returned to the screen without any bounding boxes, the threshold may be too high which is causing the bounding boxes to be thrown out and ignored.


Shown above is what the script will poop out after the validations are checked.  As you can see, multiple bounding boxes are being printed to the screen which identifies all three shapes that the classifier sees.  Ideally each of the bounding boxes around a shape will be combined into one bounding box of the highest confidence level.  In this case, there should be only three bounding boxes that are displayed which is one for each image that the network sees.


## Built With

* MATLAB 2018b


## Authors

* **Blaine Furman** 
* **Hunter Garrette** 

