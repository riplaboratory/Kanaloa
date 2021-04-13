Kevin Nguyen
nk279@hawaii.edu
Team Kanaloa VIP

14 January 2021
- First General Team Meeting
    - Talked about consent forms, design notebooks, agenda
        - additional covid form (completed after meeting)
    - New member introduction
    - Recruitment events
    - People to take over when seniors graduate
    - I will need access to Kanaloa's GitHub for software development
    - Talked about the different subsystems
    - Google Drive Organization

20 January 2021
- First Sensor Fusion Meeting
    - Ideas for "What does success look like for recruitment"
    - Look into Darknet install for Ubuntu without CUDA
    - Schedule meeting with Tyler H and grad students to help with implementing a safeguard for the code
- Sent email requesting GitHub access for Kanaloa

21 January 2021
- General Team Meeting
    - Talked about recruitment ideas
    - Categorized the ideas from each subsystem
    - Short term: email blast
    - Long term: workshops

24 January 2021
- Met with Tyler H and grad students
    - Overview of current Velocity Controller
        - Uses a Proportiaonal Integral Derivative (PID) controller
    - Deciding which direction to move forward with this
        - Use an existing package or continue working on the one made from scratch
    - Discussing how we can leave the code so that newer members will be able to easily catch up
        - Commenting on code
        - Block diagrams
        - GitHub wiki
    - Splitting up task to help get the velocity controller working well

25 January 2021
- Fork existing Kanaloa repo to personal repo
- Work on SOP (`README.md`) to more effectively use GitHub for the team
    - Organization
    - Productive
    - Consistency
    - Are we using the most out of it?
        - Branches
        - Project Boards
        - Issues
    - Overall, how can the team more effectively use GitHub in software development rather than just a code repo

27 January 2021
- Sensor Fusion Meeting
    - Team updates
    - Received feedback
        - There are currently different versions of the same code
            - We can utilize branches to keep the Main branch the most up-to-date code 
                - Branches should be used to work on code then merged into Main branch once working and done
            - [Reference Repository](https://github.com/riplaboratory/Kanaloa/tree/master/SurfaceVehicles/WAMV/Arduino/MainMega)
    - Create more specific objectives for the week

28 January 2021
- Started outline for GitHub SOP
- General Team Meeting
    - Going over Code of Conduct ideas
    - Why we joined the team
    - Long term Goals
    - Organizational Charts
        - Submissions for category names/ideas by Tuesday 2 PM
            - Make submission [here](https://docs.google.com/document/d/1lHJmt83f4D3grxKv5nFtuyYp-BHee437vsrmmoXBENU/edit?usp=sharing)

29 January 2021
- Continued work on GitHub SOP
    - Used this to create examples of issues, project boards, and branching
    - Finished Outline, need details on usage still
    - Added in resources for reference

01 February 2021
- Continued work on GitHub SOP
    - Completed Issues SOP
    - Completed Branches SOP
    - Compelted Project board SOP
    - Sent out draft to Dr. Trimble for review

03 February 2021
- Sensor Fusion Meeting
    - Team updates
    - Start working on [Darknet](https://pjreddie.com/darknet/) while waiting for response on GitHub SOP
        -  [Installation Instructions](https://pjreddie.com/darknet/install/)
        - Brief overview of what I know is that OpenCV and CUDA are optional dependencies
            - These are the reason I ran into issues previously
            - OpenCV allows support for more image types
            - CUDA makes it run faster
        - Since most members use a VM, it may be worth trying to see if it is viable without a GPU
            - My prediction is CUDA causes issues on a VM, might be worth it on for an Ubuntu host system

04 February 2021
- General Team Meeting
    - Mission Statement, finalized Meeting SOP
    - Choose specialization and project
    - Create mission and objective statement for project

08 February 2021
- Darknet installed and compiled with no issues
    - Needs to be tested to verify it works properly and if it'll work with our images and YOLO files

09 February 2021
- Confirmed Darknet is able to detect images
    - Startng to find out how to [train Darknet](https://towardsdatascience.com/custom-object-detection-using-darknet-9779170faca2) for our objects
    - Added images and labels to darknet/data directory
    - This [script](https://github.com/mukundsharma1995/yolov3-object-detection/blob/master/split_train_test.py) might come in handy to quickly make train.txt
    - Created a custom yolov3_custom_data.cfg in darknet/data/cfg

11 February 2021
- General Team Meeting
    - Still need to decide a lead for GNC
        - I sent an email out to the GNC subsystem folks
            - I became GNC lead
            - Sent out email with a when2meet so I can try to plan a meeting for everyone
    - We have TIDES at the end of this Spring 2021 semester
    - WAM-V needs to finalize mission statement
        - Met WAM-V project team and finalized mission statement
    - Create objectives statements
        - Alex S. wants us to create objectives for our subsystems then he will generalize them

14 February 2021
- Scheduled GNC meetings for Mondays at 7:00 PM
- Created WAM-V GNC ovjective 
    > The WAM-V GNC system currently lacks an autonomous drive and object classification software. GNC’s objective is to further develop the autonomous drive using a proportional–integral–derivative (PID) controller with a feedforward component and using Darknet (neural network) and You Only Look Once (YOLO) for object classification. The PID controller is in a half-working state using code built up from scratch. The objective for this component is to use the Robot Operating System’s (ROS) package to implement the controller and have a freeforward component so that it can work with it. For object classification, the neural network is partially trained with 200 images and the current documentation does not work on all devices. The neural network needs working documentation to be available for any type of machines (host or virtual) and to be set up on at least one lab computer.
- Added functional requirements for GNC's neural network
    - 02-GNC-SD: Documentation recreated w/o CUDA and OpenCV (if possible)
    - 03-GNC-SD: Darknet installed and configured on at least (1) lab computer
    - 04-GNC-SD: Implement ROS PID package
    - 05-GNC-SD: Have freeforward component for the system to work with PID

18 February 2021
- General Team Meeting
    - Reviewed mission statements
    - Went over objective statements
    - Project function requirements and success criteria

21 February 2021
- Downloaded Raymond's trained weights for Darknet from Fall 2020 Semester
    - Added it to current darknet folder
- Currently getting error `Couldn't open file: /home/kevin/darknet/labels/train/93.txt`
    - `93.txt` is being replaced by a different `.txt` file in the same directory each run
    - There is no `labels` folder in the directory, so I am not sure where its pulling this path from
- I found an alternative [Darknet installation from AlexeyAB](https://github.com/AlexeyAB/darknet)
    - Previously used [pjreddie](https://github.com/pjreddie/darknet)
    - This one seems to be working at the moment
- Started Darknet documentation
    - Need to look into what batch and subdivisions do in `custom_train.cfg` and `custom_test.cfg`

22 February 2021
- WAMV Meeting
    - Updating on our the task we've been doing
    - Tyler H. and I offered help with Ubuntu and ROS set up if they need to use it in the future
    - Yuuma asking about 3D printing
        - Referred to grad students to see if they know if Kanaloa has one
- GNC Meeting
    - Updates and Action items
    - Recommended ROS tutorials for new members
    - Provided new members the links to previous code for station keeping that they can reference
    - Provided a PowerPoint that was made on the GNC for Sensor Fusion in the previous semester
- Sent Dr. Trimble a follow up email regarding the GitHub SOP 

25 February 2021
- General Team Meeting
    - Went over success criterias and functional requirements for all projects
    - Need to rework functional requirements
    - [Brainstorming](http://rip.eng.hawaii.edu/wp-content/uploads/2017/09/designExamples_Konh_20170918.pdf) for project design

28 February 2021
- batches / subdivisions = # of images being processed 
- Batch is the quantity of images being loaded for an iteration
- Subdivision is the number of 'mini batches' that the batch will be split into
- Definitions from [this thread](https://github.com/pjreddie/darknet/issues/224#issuecomment-450406093)
    - Epoch = a pass through the full dataset
    - Batch training = update the weights after one epoch
    - Minibatch training = update the weights after a set number of samples have been seen
- [CFG Parameters in the [net] section](https://github.com/AlexeyAB/darknet/wiki/CFG-Parameters-in-the-%5Bnet%5D-section)

01 March 2021
- GNC Meeting
    - Updates and action items

03 March 2021
- Reworked functional requirements
- Started decision making matrix (DMM) for Darknet
     - Need a lot of help on this since I am not sure how to implement it in software

04 March 2021
- General Team Meeting
    - Update on action items
    - Brainstormed ideas will be gone over in project meetings
    - Decision making process: [Fundamentals of Design (Slide 51)](http://rip.eng.hawaii.edu/wp-content/uploads/2020/10/me481-conceptingModeling-2020f.pdf) and [Design Process](http://rip.eng.hawaii.edu/wp-content/uploads/2018/10/me481_designProcess_20181003.pdf)

08 March 2021
- Started making a `thruster_config.yaml` for Alex S.
    - There is a middle thruster for some reason, need to get rid of it
    - Need to add the two thrusters on the bow

11 March 2021
- Continued work on Darknet documentation
    - Added `Extra Notes` section
    - Added `Usage` section
- Ran a test with 25 images to see how long it would take to train the network
    - Stopped after a hour, this is not reliable for training right now
        - Saw that OpenCV might help with the speed. I had issues in the past but it would be worth to look into it again
        - Possibly borrow a lab laptop if I need CUDA installed
- General Team Meeting
    - Update on action items
    - Went over concept designs for WAM-V
    - Next steps to take
    - Kai J suggested to use [this](https://www.hawaii.edu/its/ci/xcat/)

23 March 2021
- Dr. Trimble responded regarding SOP
    - For now, we will not utilize project boards since we are currently using Google Sheets for Action Items
        - Archive what I wrote somewhere and reference it to Google Sheets instead
        - Possibly get rid of issues since we don't need that since it falls in line with the action items issue
        - This is because with more place to reference, it becomes annoying and harder to manage
    - Create template comment blocks with original author, created, modified by, last modified, etc.
        - This will rid it of the multiversion issue that is currently happening inside the repo
- Kai J. provided me access to Kanaloa Repo
    - Need to move all my work over to it
        - Previous Documentation
        - Engineering Journals

25 March 2021
- Created a `update-github-sop` branch on `riplaboratory/Kanaloa` repository
- Moved all my work from my personal repository to branch `nk279-work`
    - Engineering Journals, Documentation, Code, Docker submissions
- General Team Meeting
    - Update on action items

29 March 2021
- GNC Meeting
    - 2 members and grad students were not able to make it due to other commitments
    - Quick meeting, went over updates and action items

1 April 2021
- Started on Code Documentation Templates
    - Needed: C, Python, Matlab, Arduino
- Kai J. suggested adding how to find previous versions of code in the SOP
    - Click on the file and then `history` to see all changes made to that specific document
- General Team Meeting
    - Updates and Action items

3 April 2021
- Added `Version History` to `README.md`
- Create Code Documentation Templates for:
    - C
    - Python
    - Arduino

4 April 2021
- Create Documentation Tempalte for Markdown
- Sent email to Dr. Trimble to Review

5 April 2021
- Dr. Trimble agrees with the created templates 
    - Need to think on what should use the in-code versioning and what needs manual versioning (in file name)
        - Manual Versioning:
            - Configuration files
            - Documents (PDFs, TXT, Schematics)
        - In-Code Versioning:
            - Software for vessels
                - Drive Controller
                - Way Point Navigation
                - Station Keeping

7 April 2021
- Emergency WAM-V meeting
    - I will take over on managing meetings for WAM-V due to Alex S. being on medical leave

8 April 2021
- General Team Meeting
    - Updates and action items
    - Dicussions on CoE banquet
    - Need to organize WAM-V meetings, there have not been any minutes or records of them
- Sent out email to WAM-V team to schedule a meeting for FR reviews

12 April 2021
- Picked up MSI laptop
    - Transferred work from my VM onto GitHub to put onto MSI laptop (riplaboratory/Kanaloa/Projects/Darknet/darknet)
        - cfg
        - custom_data
        - data
        - images
        - labels
        - weights
- GNC Team Meeting
    - Updates and action items