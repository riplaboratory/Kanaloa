## The files in this directory were added to the Docker image for submission
You can pull my image using
```
docker pull kvnng/kanaloa-vorc:v1
```

Follow my [docker documentation](https://github.com/kvndngyn/KanaloaWork/blob/main/2020/Docker.md) for more information regarding `ros_entrypoint.sh` and `run_my_system.bash`

In [kanaloa_vorc/src/kanaloa_pkg/scripts](https://github.com/kvndngyn/KanaloaWork/tree/main/2020/vorc_submissions/kanaloa_vorc/src/kanaloa_pkg/scripts) contains `task.py` which is as follows
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
			station_keep()
			current_task = "perception"

	elif task_name == "gymkhana":
		if current_task != "gymkhana":
			station_keep()
			current_task = "gymkhana"

def subscriber():
		rospy.Subscriber("/vorc/task/info", Task, assign)
	
if __name__ == '__main__':
	rospy.init_node("Team_Kanaloloz_VRX")
	while not rospy.is_shutdown():
		subscriber()
		rospy.spin()

```
This script is what decides which code to execute from our kanaloa_pkg. For example, if the trial being ran is stationkeeping, it will call `station_keep()` to run that command. I will break down the code further for easier understanding.

#### assign(data)
```python
from wayfinding import WAMV_Way_Point as wayfind
from station_keeping import WAMV_Way_Point as station_keep

def assign(data):

	global current_task

	task_name = data.name

	if task_name == "stationkeeping":
		if current_task != "stationkeeping":
			station_keep()
			current_task = "stationkeeping"
```
+ `global current_task` is a global variable
+ `task_name = data.name` is accessing the name key of the given data, in other words, the task of the trial
+ `current_task` is at first `none` but is later set to the string of the task
+ The nested `if statements`
    + The outer `if statement` matches `task_name` to 'stationkeeping' to see if they match
    + The inner `if statement` makes sure that the `current_task` does not match that of the task being ran
        + If this was not here, `station_keep()` would constantly be called as the publisher constantly outputs the task

#### subscriber()
```python
import rospy
from vrx_gazebo.msg import Task

def subscriber():
		rospy.Subscriber("/vorc/task/info", Task, assign)
```
+ `rospy.Subscriber()` allows us to subscribe to a `rostopic` and use that data to decide the current task