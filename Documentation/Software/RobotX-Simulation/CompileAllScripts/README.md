# Compiling all scripts with task.py
The script `task.py` identifies the task the WAM-V is currently expected to perform, and runs the appropriate script to carry out that task. 

## Running task.py
__1. Navigate to the scripts directory__

_All scripts used in the 2019 VRX competition are located in the following directory:_

	cd kanaloa_vrx/src/kanaloa_pkg/scripts

__2. Run the script__
 
 Currently, `station_keeping.py`, `wayfinding.py`, and `nav_channel.py` can all be run using `task.py`.

	python task.py

## Adding new tasks and scripts
__1. Determine name of task used by VRX__
In the competition, the name of the task currently being performed is published by the `/vrx/task/info` topic. Should VRX task names be added or changed in the future, they can be found using the following ROS command (while the environment for that particular task is currently running):

	rostopic echo /vrx/task/info

__2. Edit `task.py`__
Once found, the task name can be added into the script following the same format as the other tasks.

	elif task_name == "<task_name>":
		if current_task != "<task_name>":
			<task_function>()
			current_task = "<task_name>"

Because the `/vrx/task/info` topic will continuously publish the name of the task being performed while the simulation is running, the current_task variable was introduced to prevent the task function from repeatedly being started over.

__2.1 Import the task function/classes from their respective python scripts__
If multiple scripts utilize the same class, they can be renamed as was done with station_keeping and wayfinding.

	from wayfinding import WAMV_Way_Point as wayfind
	from station_keeping import WAMV_Way_Point as station_keep
