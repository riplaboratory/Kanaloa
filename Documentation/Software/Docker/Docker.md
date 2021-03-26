## Getting Started
Create a docker account at [Docker Hub](https://hub.docker.com/) and get started with their [documentation](https://docs.docker.com/).

Get familiar with the command line interface (CLI). There are a lot of useful tutorials, documentation as well as games to help you. Here is an example of a game: [Terminus game](http://web.mit.edu/mprat/Public/web/Terminus/Web/main.html)

## Terminology
### Image
An image is like a picture of a Docker virtual machine at a specific point in time. This can be shared with others so that work can continue from that point. This can be used as a foundation for others to build off of. It is your immutable file that contains all the source code, libraries, dependecies, tools of your application. 
### Container
A container is an instance of the image. From just one image, you can create an infinite amount of containers of instaces of it. You can the build or continue developing your application starting off where the machine left. Your container is a docker virtual machine running your application where you can continue working on it. You can later push this to make it become an image that others can use to continue off of.

## What You Should Know
View all images
```
docker images
```
Remove an image
```
docker rmi <image>
```
Remove all images
```
docker rmi $(docker images -aq)
```
View running containers
```
docker ps
```
View all containers
```
docker ps -a
```
Remove a container
```
docker rm <container>
```
Remove all containers
```
docker rm $(docker ps -aq)
```
Commit a container
```
docker commit <container name> <docker username>/<repository>:<tag (default: latest)>
```
Push the changes
```
docker push <docker username>/<repository>:<tag (default: latest)>
```
If you want to run multiple sessions of the same container, in a new terminal do
```
docker exec -it <container name> bash
```
If you want to override the entrypoint script and get back into a Bash session do
```
docker run -it --name <container name> --entrypoint bash <docker username>/<repository>:<tag>
```

## ROS and Docker
#### Initial Setup
Get the ROS Melodic base image
```
docker run --name my_container -it ros:melodic-ros-base
```
+ This pulls `ros:melodic-ros-base` image from Dockerhub and creates a container called `my_container`
+ `-it` starts an interactive Bash session
+ If you have not yet pulled the image, this will also pull it

You should then be in the bash session for `my_container`

![](./images/bash_session.png)

Make sure everything is up to date
```
apt update && apt install -y vim
```
+ `vim` can be replaced with the text editor of your choice

You will want to edit `ros_entrypoint.sh` (the script that will be ran on startup) so that it sources `setup.bash` and calls another script to run your system
```bash
#!/bin/bash
set -e

# setup ros environment
source "/kanaloa_vorc/devel/setup.bash"

echo "Doing Kanaloa Things..."            

/run_my_system.bash
echo "We Did it, that's it!"
```
+ Note that you want to only be sourcing one path. You package much be within the same parent folder as your `setup.bash`. If you try to source multiple enviornments in your entrypoint script, you will only be sourcing the first line and the others will be ignored.

Next we want to create a `run_my_system.bash` script using `vim run_my_system.bash` and enter the following
```bash
#!/bin/bash

# Create ros master if not already started
rostopic list > /dev/null 2>&1
retVal=$?
if [ $retVal -ne 0 ]; then
    roscore &
    echo "Wait for 5s to allow rosmaster to start"
    sleep 5s
else
    echo "rosmaster already setup"
fi

# Enter scripts or launch files to call below
python /kanaloa_vorc/src/kanaloa_pkg/scripts/task.py
```
For this example we will be using Kanaloa's `task.py` which will be shown later in the documentation.

As shown from `ros_entrypoint.sh` we sourced from a `vorc_ws` along with `kanaloa_vrx` which are files that you will need to add yourself. To add `vorc_ws` you can follow this [tutorial](https://github.com/osrf/vrx/wiki/tutorials-SystemSetupInstall). Note in our case, we are in the root `/` directory and not the home `~` directory. When following the tutorial, it may be convenient to replace `~` with `/` so that all your workspaces are easy to find.

To get `kanaloa_vrx`, you can either use `git clone` or `docker cp`. In my case, since there was no repository for just the `kanaloa_vrx` workspace, I used `docker cp`. To use this command, you will want to first exit your Bash session by either typing in `exit` or using the keyboard shortcut `ctrl + P + Q`.

Navigate to where the workspace is on your system and type in the following command
```
docker cp <folder/file> <container>:/
```
This will copy your folder/file from your system into the root directory of your docker container.

Now your container is all done with the basic set up.

#### Kanaloa's Python Script
`task.py` is a script that helps decide what Python code should be ran. The following is the script
```python
#!/usr/bin/env python
# license removed for brevity

import rospy
from vrx_gazebo.msg import Task
from wayfinding import WAMV_Way_Point as wayfind
from station_keeping import WAMV_Way_Point as station_keep

current_task = None

def assign(data):
	global current_task

	task_name = data.name

	if task_name == "stationkeeping":
		if current_task != "stationkeeping":
			station_keep()
			current_task = "stationkeeping"

	elif task_name == "wayfinding":
		if current_task != "wayfinding":
			wayfind()
			current_task = "wayfinding"
        
    elif task_name == "perception":
        if current_task != "perception":
        # TODO: Add Code
        current_task = "perception"

    elif task_name == "gymkhana":
        if current_task != "gymkhana":
        # TODO: Add Code
        current_task = "gymkhana"

def subscriber():
	rospy.Subscriber("/vorc/task/info", Task, assign)
	
if __name__ == '__main__':
	rospy.init_node("Team_Kanaloloz_VRX")
	while not rospy.is_shutdown():
		subscriber()
		rospy.spin()
```

It is important that somewhere in this code that we have a `rospy.Subscriber()` for `/vorc/task/info` as this is used to find out the name of the trial currently being ran. Be sure that your entrypoint scripts are sourcing properly or else you will get an `invalid syntax` for `from vrx_gazebo.msg import Task`. Currently this code is calling the function `assign()` which and passes the data into it. `assign()` then access `data.name` and assigns it to `task_name` which is used to decide the task being done to decide which code to run. However, during the simulation, the data is constantly being repeated and we do not want to constantly run our code from the beginning. To avoid this issue, we assign `current_task` so tht we can created a nested `if` statement. this makes sure that we are only running code if the task was not yet being executed. The rest of the code can be found at [KanaloaWork/2020/vorc_submissions/kanaloa_vorc](https://github.com/kvndngyn/KanaloaWork/tree/main/2020/vorc_submissions/kanaloa_vorc).

## Commit and Push
Now that your workspace is done being set up, you will want to commit your work and push the container as an image to your repository. If you have not yet created a repository, it should be created when making the push. First make sure you login to your docker account by doing
```
docker login
```

#### Commit
To commit your changes you will want to do
```
docker commit -m "Your message here" <container> <docker username>/<repository name>:<tag>
```
+ `-m "Your message here"` is optional to add
+ `<container>` can either be the container's name or ID 
+ `<repository name>` is the name of your docker repository
+ `<tag>` will is used to create another instance of your image
	+ if this is ommited, then it will default to the `latest` tag
+ Here is an example of what I would do
```
docker commit my_container kvnng/kanaloa-vorc:v1
```
You should get a random string of characters similar to
```
sha256:3a4c068027d428f33655d45da440d3ebc558816e2a61091aba88af41fee92610
```
#### Push 
After commiting, you will not want to push the changes so that is is accessible online. To do so
```
docker push <docker username>/<repository name>:<tag>
```
+ `<repository name>` is the name of your docker repository
+ `<tag>` will is used to create another instance of your image
	+ if this is ommited, then it will default to the `latest` tag
+ Here is an example that I would do
```
docker push kvnng/kanaloa-vorc:v1
```
You should get a similar message to this
```
The push refers to repository [docker.io/kvnng/kanaloa-vorc]
40b56e552a35: Layer already exists 
ac199f5fc8c0: Layer already exists 
7c0b678c2e8a: Layer already exists 
28881e12e672: Layer already exists 
86f67e37a47a: Layer already exists 
c82a09854d2b: Layer already exists 
394548e0fff1: Layer already exists 
0647933524d9: Layer already exists 
fc279bfb4009: Layer already exists 
f020c1620d91: Layer already exists 
6663ec90ed5e: Layer already exists 
19331eff40f0: Layer already exists 
100ef12ce3a4: Layer already exists 
97e6b67a30f1: Layer already exists 
a090697502b8: Layer already exists 
v1: digest: sha256:8e49afff6228614a01331706243646fccd616a30d2d7818950b5f1ed7fc91d2d size: 3456
```

Once that is all done, you can check [Docker Hub](https://hub.docker.com/repositories) to make sure that all changes have gone through. To test your evaluations, OSRF provides [this repository](https://github.com/osrf/vrx-docker) with how they run the simulation for teams. Make sure to check their branches to see if they have anything for the ongoing competition as the master branch is for VRX.

## Troubleshooting
#### vorc-network issue
If you receive the following error message, it may be due to the use of a previous competitions automated evaluation. In my case, I had an issue with the connecting to the `vorc-network` since I currently had the `vrx-network`.
```
docker: Error response from daemon: network vorc-network not found.
ERRO[0000] error waiting for container: context canceled
```
To fix this, double check if you have `vrx-network`
```
docker network ls
```
Once you confirm that it is the issue run the following command to remove
```
docker network rm vrx-network
```

#### Unsure if your `ros_entrypoint.sh` script is running
If you are unsure if your `ros_entrypoint.sh` script is running, you can use the following to check
```
docker ps -a
```
This allows you to check your containers and information. The container holding you workspace should have `/ros_entrypoint.sh` under the `Command` column. Another tip is to have your script echo a string of text in your terminal. You can add the following to `ros_entrypoint.sh` in your container
```
echo -e "Some text"
```
When running the container, you should be able to see "Some text" appear in your terminal.

#### Docker entrypoint command wrong
When doing `docker ps -a` you should see all your containers
```
CONTAINER ID        IMAGE                                        COMMAND                  CREATED             STATUS                         PORTS               NAMES
99d43fce1d67        kvnng/vrx-competitor-example:my_image_2      "/ros_entrypoint.sh"     4 minutes ago       Exited (2) 4 minutes ago                           my_container_3
10c410d57263        kvnng/vrx-competitor-example:my_image        "bash"                   2 hours ago         Up About an hour                                   my_container_2
```
In the command column, for the submission to work, it should be running your `/ros_entrypoint.sh`. If it is anything different, then it may be due to running
```
docker run -it --name <container name> --entrypoint bash <docker username>/<repository>:<tag>
```
What this does is create a new container and overwrites the entrypoint command to be Bash. To fix this, you will want to finish making changes in this Bash session then commit and push the change with
```
docker commit <container name> <username>/<repository>:<tag>
docker push <username>/<repository>:<tag>
```
Once you do that, you will want to run that container to have the entrypoint command to be your `/ros_entrypoint.sh` by
```
docker run -it --name <container name> --entrypoint /ros_entrypoint.sh <docker username>/<repository>:<tag>
```
This will create a new container that uses your `/ros_entrypoint.sh` as your entrypoint command. You can then commit and push this image to your docker repository.

## Helpful Guides
[Docker Documentation](https://docs.docker.com/)

[VORC Automated Evalutations](https://github.com/osrf/vrx-docker/tree/vorc)

[OSRF's Tutorial](https://github.com/osrf/vorc/wiki/creating_docker_image)






