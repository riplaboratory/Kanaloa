# Compiling all scripts with task.py
The script `task.py` identifies the task the WAM-V is currently expected to perform, and runs the appropriate script to carry out that task. 

The six tasks of the VRX competition are:
1. Stationkeeping
2. Wayfinding
3. Landmark Localization and Characterization
4. Traverse Navigation Channel
5. Dock
6. Scan and Dock

The tasks submitted for VRX 2019 were stationkeeping, wayfinding, and navigation channel. These three tasks can be run using the `task.py` script.


## Prerequisites

- Download kanaloa_vrx workspace located [here](https://github.com/riplaboratory/KanaloaVrx2019/tree/master/kanaloa_vrx)
- Install the VRX simulation, [instructions here] link to marsa and jdy tech doc

<hr>

## Running task.py
__1. Open the environment you would like to run a task for in Gazebo__
1.1 Open a terminal window and enter the following command:

`WORLD=stationkeeping0.world`
      
Notice we have set `WORLD` to be the station-keeping environment, but this can be changed to any other environment found in the following directory:

 `cd vrx_ws/src/vrx/vrx_gazebo/worlds`
 
 1.2 Open the Gazebo simulation with the following command:

`roslaunch vrx_gazebo vrx.launch verbose:=true \
      paused:=false \
      wamv_locked:=true \
      world:=${HOME}/vrx_ws/src/vrx/vrx_gazebo/worlds/${WORLD} urdf:=$HOME/my_wamv/my_wamv.urdf`


__2. Navigate to the scripts directory__
In a new terminal tab, enter the following command:

	cd kanaloa_vrx/src/kanaloa_pkg/scripts

__3. Run the script__

	python task.py
	
After entering the command, a task will be performed depending on the environment that is being ran. If using the command provided in step 1.1, the WAMV will begin to stationkeep. Again, the only worlds that a task can currently be performed for are stationkeeping, wayfinding, and navigation channnel.
	
<hr>

## Adding new tasks and scripts
__1. Determine name of task used by VRX__

In the competition, the name of the task currently being performed is published by the `/vrx/task/info` topic. Should VRX task names be added or changed in the future, they can be found using the following ROS command (while the environment for that particular task is currently running):

	rostopic echo /vrx/task/info
	
As an example, if you were within a docking task world using WORLD=dock0.world(see "Running task.py", Step 1 above), running the above command should show:

	name: "scan_dock"
	
So we know the name of the task is `scan_dock`.

__2. Edit `task.py`__
2.1 Open task.py using any code editor. If Sublime  is installed on your computer, you may use the following command while in the `kanaloa_vrx/src/kanaloa_pkg/scripts` directory:

`subl task.py`

2.2 Import the task function/classes from their respective python scripts 
For example, let's say you wanted to add a new script that performs the docking task. Your script is named `dock.py` and the function used within the script is called dock_main(). To import a function to `task.py`, you would use the following format:

`from <script_name> import <function/class_name>`

so the dock_main() function can be imported using:

`from dock import dock_main`

If multiple scripts utilize the same class, they can be renamed as was done with station_keeping and wayfinding, exemplified below:

`from wayfinding import WAMV_Way_Point as wayfind`
`from station_keeping import WAMV_Way_Point as station_keep`


2.3 The task name found in step 1 can be added into the script following the same format as the other tasks:

	elif task_name == "<task_name>":
		if current_task != "<task_name>":
			<task_function>()
			current_task = "<task_name>"
			
The hypothetical `dock.py` script from step 2.2 can then be added using the following code:

	elif task_name == "scan_dock":
		if current_task != "scan_dock":
			dock_main()
			current_task = "scan_dock" 

######Note: Because the `/vrx/task/info` topic will continuously publish the name of the task being performed while the simulation is running, the `current_task` global variable was introduced to prevent the task function from being started over repeatedly.

__The new script will now be successfully added to task.py__
