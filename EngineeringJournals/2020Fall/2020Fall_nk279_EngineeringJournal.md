Kevin Nguyen
nk279@hawaii.edu
Team Kanaloa VIP

08 August 2020
+ Had first Sensor Fusion meeting
    + Introductions
    + ROS tutorials
    + Simulation tutorials

09-14 August 2020
+ Completed ROS Tutorials:
    + Understanding ROS Nodes
    + Understanding ROS Topics
    + Understanding ROS Services and Parameters
    + Using rqt_console and roslaunch
    + Creating a ROS msg and srv
    + Writing a Simple Publisher and Subscriber (Python)

15 August 2020
+ Sensor Fusion Meeting
    + Discussing GNC system
    + Discussing how to move WAM-V:
        + Send motor commands to thruster
        + Send voltage to thrusters
        + Change positions and angling of thrusters

22 August 2020
+ Meeting postponed to 25 August 2020 since a lot of people were busy

25 August 2020
+ Sensor Fusion Meeting
    + Continued disscusion of the GNC system
        + Creating a plan to break it down to work on it

26-28 August 2020
+ I was not assigned any task yet as most of it was continuing off last semester
+ I went through the ROS tutorials again to make sure I did not miss anything
+ Started familiarizing myself with Python so that I could read the code previously developed

29 August 2020
+ Sensor Fusion Meeting
    + Went over ROS and Kanaloa terminology
    + Made sure everyone had the simulation set up on their device 
    + Overview of how to launch the simulation with custom thruster and sensor configs
    + Overview of rostopics and rosbags
    + Overview of how to make a Python script executable using `chmod +x <filename.py>`
    + Assigned exercises for the team relating to the meeting
+ Exercise
    + Created a custom `sensor_config.yaml` and `thurster_config.yaml` file.
    + Created a custom `my_wamv.urdf`
    + Ran thruster config in simulation using RQT

06 October 2020
+ Sensor Fusion Meeting
    + GNC was broken down into different task
        + Object Classification
        + Object Detection
        + Localization
        + Mapping
        + Behaviour Planning
        + Trajectory Planning
        + Thruster Controller
        + Math Modeling (for navigation)
    + Overview of what each sub-team would be expected to accomplish
    + Tasked to decide what we want to work on

07 October 2020
+ Message sent out to choose teams
    + I chose to work on Object Classification

08 October 2020
+ Virtual Ocean Robotics Challenge (VORC) announced to be held in Decemeber

13 October 2020
+ Sensor Fusion Meeting
    + Update on each member's respective task
    + More information regarding VORC
    + Two meetings would be held each week to meet competition deadline in December 
        + I am not able to attend it as I have class during the time
        + My updates will be provided via group chat

14 October 2020
+ Asked for previous work with Object Classification

15-19 October 2020
+ I was provided the Google Drive with documentation on what was done last semester
+ Looked through Darknet installation
+ Looked through labelImg installation
+ I was provided explanation the plan of what would be done
    + Objects in the image would be labeled
    + This will create a YOLO file
    + The YOLO file will be given to the neural network to train

20 October 2020
+ Discussed more about task for VORC
+ Discussed last year's VRX competition
    + Issues with Docker

21-26 October 2020
+ Started setting up Darknet on Ubuntu
    + Issues with OpenCV
        + I can not find the files that the documentation said there should be
        + I tried looking into the issue but could not find an answer
        + Putting aside temporarily to see if other members may know what to do
    + Continued with Darknet installation
    + Set up Anaconda on Ubuntu system
    + Issues with CUDA
        + I am not able to find the `.run` file
+ Ubuntu crashed and would not boot anymore
    + Temporarily not working on Darknet due to frustration with numerous complications
+ I started to look into Docker since I worked with a similar application

27 October 2020
+ Sensor Fusion Meeting
    + Team just updating where everyone is at

28 October - 02 November 2020
+ Luckily I am using a VM and was able to just delete the boot looped one
+ Started a new Ubuntu 18.04 VM
    + Reinstalled ROS Melodic
    + Issues persist with Darknet and OpenCV

03 November 2020
+ Sensor Fusion Meeting
    + Team just updating where everyone is at
+ Shared my issues with the other members
    + Seems that there were issues for them as well

05 November 2020
+ Found out an article to fix my issue with CUDA (a step for Darknet Installation)
    + Used `wget https://developer.nvidia.com/compute/cuda/10.1/Prod/local_installers/cuda_10.1.105_418.39_linux.run`
    + I was able to complete Darknet installation
    + OpenCV still having issues
+ We decided that my time would be better spent labeling images since a member already had the neural network set up on their system
+ Issues with labelImg on Ubuntu

07-09 November 2020
+ Decided to set up an Anaconda environment on Windows and use labelImg through that
+ I was provided explanations on how to use labelImg
+ Issues with labelImg
    + When assigning a save directory, program crashes and gives the following error:
      ```
      Traceback (most recent call last):
      File "labelimg.py", line 1355, in openNextImg
        self.loadFile(filename)
      File "labelimg.py", line 1096, in loadFile
        self.showBoundingBoxFromAnnotationFile(filePath)
      File "labelimg.py", line 1114, in showBoundingBoxFromAnnotationFile
        filedir = filePath.split(basename)[0].split("/")[-2:-1][0]
      IndexError: list index out of range
      ```
    + Looked through labelImg GitHub issues to find a solution
    + No solution provided helped, but I was able to determine that it was due to a bug in the code
+ Continued labeling images
    + No crash happens as long as I did not assign a save directory

10 Novemeber 2020
+ Sensor Fusion Meeting
    + General team updates as usual

11-16 November 2020
+ Continued labeling images

17 Novemeber 2020
+ Checked through labelImg GitHub to see there were any updates
+ A commit was made recently, I cloned the new update
    + This fixed my issue with the crash when choosing a save directory
+ Finished labeling my first 50 images, downloaded the next 50 I would work on
+ Sensor Fusion Meeting
    + General team updates
+ Continued labeling images
    + The 50 new ones did not have much obstacles so I was able to complete these the same night

23 November 2020
+ I submitted to GitHub for VORC
    + The repo was forked, and in that, I just added a text file with the video link

24 November 2020
+ Between the others and I, 200 labeled images were created
+ Sensor Fusion Meeting
    + General team updates
    + I am now assigned to work on the Docker submission
+ Provided the link to where the labeled images were

25-29 November 2020
+ Set up Docker on my system
+ Read through documentation to try and find out how to create an image
+ Confusion between containers and images
    + Images is the base of an application. This can be created and used to build off of. For VORC, we are pulling their base image and building off that to create our image for submission
    + Containers are an instance of that image. All work is done locally on the system and will later be pushed into an image
+ Dug through OSRF (the organization holding the competion) and was able to find our previous submission that had issues
+ I also pulled an image from a team that qualified so I can see how it should work

01 December 2020
+ Sensor Fusion Meeting
    + General Team Updates
+ Found out that I messed up 100 labeled images
    + Found a python script to go through all my text files and replace the object's number (first column of the YOLO file)
      ```python
      import os, re

      path = '/Users/kvndn/labelimg/labeled'
      directory = os.listdir(path)
      os.chdir(path)
      print(directory)

      # regex for all lines but the first /(?<=\n)3/ where 3 can be replaced with 1 or 2
      # regex for first line only /^3/ where 3 can be replaced with 1 or 2

      for file in directory:
         open_file = open(file,'r')
         read_file = open_file.read()
         regex = re.compile('(?<=\n)3')
         # regex = re.compile('^3')
         read_file = regex.sub('2', read_file)
         write_file = open(file,'w')
         write_file.write(read_file)
+ Added this script to the object classification drive
+ Seems that everything needed to be redone
    + Ran the code on all 200 files
+ Created a quick documentation in the drive for others to avoid this mistake in the future
    + Also gives a rundown on how the code is to be used
    + This could probably be optimized but is currently out of my abilities and time

02-06 December 2020
+ Set up OSRF's VRX automated evaluations from `vrx-docker` on GitHub
    + This allows me to test the image I created to see if it works
    + Using our old image, I was not able to get it working
    + Creating a new image following their documentation I got something running
    + Using `docker ps -a` to view all containers I had, I realized that our old image differed from what I had and from what other teams had
    + Under the `Command` column, should have been the startup scripts but the old image was running some `fixuid` command instead

07 December 2020
+ Found out that they had a branch for VORC automated evaluations
+ There is an issue where it produces this error:
  ```
  docker: Error response from daemon: network vorc-network not found.
  ERRO[0000] error waiting for container: context canceled 
  ```
    + I believe this is something wrong with one of their scripts
+ Went back to just making sure our docker image works by just working within the VRX workspace
+ Others and I were able to discover that this was how things were being processed when images are being evaluated
    + `ros_entrypoint.sh` > `run_my_system.bash` > `task.py`
        + `ros_entrypoint.sh` is a script that sources:
            + `/opt/ros/melodic/setup.bash`
            + `/vorc_ws/install/setup.bash`
            + `/kanaloa_vrx/devel/setup.bash`
            + We know it gets to this as the Kanaloa logo is echoed to our terminal during the trial
        + `run_my_system.bash` is a script that calls `task.py`
        + `task.py` is our script that determines which python code we should run
            + It is subscribed to `/vorc/task/info` which allows us to add `from vrx_gazebo.msg import Task`
                + This is important as it allows us to get the name of the task being ran
            + Still need to edit the script to be used for VORC instead of VRX
+ After a lot of trial and error, we got a docker image working
+ I am 90% confident that Docker will not be the issue with our submission this year
+ Need to ask team tomorrow if we have anything developed for VORC or are we going to use VRX's stationkeeping and wayfinding
+ Submissions are open and are due 11 December 2020 9:00 PM HST

08 December 2020
+ Final Sensor Fusion Meeting
    + General team updates
    + Asked whether we have code developed for VORC
    + Decided we will use VRX's code and just make sure that it works in VORC
        + Hoping little configuration will be needed
+ Sent out email asking how I could get the VORC automated evaluations working
+ Planning to work with the others on creating a final submission 09 Wednesday 2020
    + We are planning to get a submission completed by this date
    + I am busy with classes Thursday
    + Friday will only be for working out kinks in the submission
+ Created [Docker documentation] (https://github.com/kvndngyn/KanaloaWork/blob/main/2020/Docker.md)
+ Added [`kanaloa_vrx` workspace](https://github.com/kvndngyn/KanaloaWork/blob/main/2020/kanaloa_vrx)

09 December 2020
+ Mabel Zhang from VORC emailed back with help
    + Seems that since I ran the VRX Automated Evaluations, I was having the issue on my machine
    + Proposed solution to run `docker network ls` to see if vrx-network is running
    + If vrx-server exist, run `docker network rm vrx-network`
    + Hopefully this allows the `vorc-network` to be made
    + Updated Docker documentation with this information
+ Need to update scripts in docker container back to VORC and remove any lines related to VRX
    + May need to refactor current `kanaloa_vrx` workspace for VORC
+ Able to run VORC simulation
+ Found out we can only source one path in the startup script.
    + Refactored workspace for Kanaloa's old VRX code to work with VORC
    + There was a lot of differences so although docker works, the code does not
+ We can conclude that the docker images works and that we can compete. However, since nothing was developed for VORC, whatever little refactoring we were able to do from old VRX code is all we can submit. CoRa's configurations are completely different from what was developed for VRX.
+ Created documentation for `relabelScript.py`

10 December 2020
+ Submission was using old code still subscribed to VRX API, so I updated it for VORC API
+ Pull request for VORC event was approved and commited into their branch
 



