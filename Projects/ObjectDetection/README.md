# Documentation

This directory contains documentation on visualizing and interpreting PointCloud2 data.

## Installation
This tutorial assumes that you already have the following installed:
+ Ubuntu 18.04
+ ROS Melodic
+ VRX Simulation

You will then need to install the following 
```
sudo apt install -y build-essential cmake cppcheck curl git gnupg libeigen3-dev libgles2-mesa-dev lsb-release pkg-config protobuf-compiler qtbase5-dev python3-dbg python3-pip python3-venv ruby software-properties-common wget ros-melodic-joint-state-publisher

```

## Launching RViz
First, if you have not already, source your workspace
```
source /vrx_ws/devel/setup.bash
```
Then launch Gazebo (in this case, I am using the example enviornment)
```
roslaunch vrx_gazebo vrx.launch
```
Then launch RViz
```
roslaunch wamv_gazebo rviz_vrx.launch
```

In here, you want to make sure that you have PointCloud2 added and it is subscribed to the right topic. 
If it is not yet added, then you can click on `Add` above the camers on the left pane and select `PointCloud2`. On success,
if you move the WAM-V, then you should start seeing a screen similar to the one below.

![Lidar in Rviz](images/rviz-pc2-lidar.png)

### Troubleshooting
+ If `numpy` module is not found, you can install it by doing `pip3 install numpy`
+ You may run into issues launching Gazebo due to a "symbol lookup error". If that is the case, run `sudo apt upgrade libignition-math2`

## Code Usage
An example of how to interpret data can be found in `lidarMsgs.py`. Here is a small break down of what the code does:

### Importing Libraries
This is all you need to simply access and interpret the data in Python
```py
import rospy
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2
```
+ `rospy` is the ROS Python library
+ `sensor_msgs.pointcloud2` is a library to take in and read the PointCloud2 data
+ `from sensor_msgs.msg import PointCloud2` is the data itself

### Interpreting the data
To interpret the data, we can iterate a loop to go through each field by doing
```py
for p in pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True):
```
This allows us the read the points in the PointCloud2 data and we are specifically looking in the `fields` field for `x`, `y`, `z`
Then to do something with the points, you can access by doing a line similar to this:
```py
print("x: %f\ty: %f\tz: %f" % (p[0],p[1],p[2]))
```
Using `p[0]`, `p[1]`, `p[2]`, we can access `x`, `y`,` z` respectively. This snippet shows how to print it so you can use it as a way to see what
kind of data is being given, however, it can be used for more advance cases.

### Putting it all together
To put it all together, you can run your function in the main function
```py
if __name__ == '__main__':
    try:
        # Topic of your pointcloud2 data
        # You can find this from `rostopic list`
        topic = "/wamv/sensors/lidars/lidar_wamv/points"
        print(f"Subscribing to {topic}")
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber(topic, PointCloud2, printLidar)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
```
+ As long as there is no error or interrupts (such as `ctrl+c`), the `try` will continue to loop because of `rospy.spin()`. 
+ `topic` is the PointCloud2 topic that your script will be subscribing to. This may vary depending on your publisher, but the `vrx` example publishes
`/wamv/sensors/lidars/lidar_wamv/points`
+ `rospy.init_node('listener', anonymous=True)` initiates a node to listen to the publisher.
+ `rospy.Subscriber(topic, PointCloud2, printLidar)` subscribes to the topic with PointCloud2 data and runs the `printLidar` function

### Example
If you want to run the example, open a new terminal and change directories to where the file is located. In my case, I stored it in `scripts` so I used
```
cd ~/vrx_ws/src/scripts
```
Then compile the script by doing
```
chmod +x lidarMsgs.py
```
Now, if you have not already done so, launch Gazebo, and then run the following command to run the script
```
python3 lidarMsgs.py
```

## Further Development
After further research, it seems I strayed off slightly into the wrong direction. While PointCloud2 data is useful in RViz when it comes to live object detection,
it does not help much in code at first glance since the data is given in pixel/image coordinates in reference to the frame of the image rather than real world
coordinates, or the xyz in meters. There are two ways that I have researched to get a real world measurement, and it is by either converting to 
PointCloud (pcl) or to LaserScan.

(Refer to `4` in resources) There may be simpler implementations, but if I were to approach this issue again with the knowledge I know, I would want to convert to
LaserScan data. by doing so, you will be able to get a [`ranges`](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html) field which is 
an array of data for distance to an object in meters. A laser scan ignores the height of an object so you have are only looking at the object in 2D (distance and width). 
The `ranges` array provides multiple points along the width, so taking the center of the range you would be able to get the center of the object from my nderstanding. 
If that is the case, then find the middle value should be fairly simple. Pseudocode for it would be: 
```py
middle = len(array) / 2 # ex. len(array) = 10 so then 10 / 2 = 5
center_of_obj = array[middle] # which would be used to access array[5]
```

If you would like to take on the challenge of converting to pcl, there would be a lot of math involved. Starting off, you can refer to `5` in resources. They provide
an answer similar to what I did, but instead use a numpy array in their conversion. Since I have not looked into this enough to understand the math well enough to
explain, you can reference `6` in resources for a detailed information on how you would use the pixel/image coordiantes to get real world coordinates.

Ideally, all this information from the LiDAR would be used alongside an object classifier in order to map out the surroundings of the WAM-V. This map would then be used 
to help with obstacle avoidance and planning for the WAM-V to make decisions autonomously.

### Resources
1. [sensor_msgs.point_cloud2 Namespace Reference](http://docs.ros.org/en/noetic/api/sensor_msgs/html/namespacesensor__msgs_1_1point__cloud2.html#afaff776c6be14a36216560613dcd50df)
2. [How to transform PointCloud2 with TF?](https://answers.ros.org/question/9103/how-to-transform-pointcloud2-with-tf/)
3. [[ROS Q&A] 120 - How To Convert a PointCloud Into a Laser Scan](https://www.youtube.com/watch?v=IFNikTHN1pk&ab_channel=TheConstruct)
4. [[ROS Q&A] 040 - How to check the distance to an obstacle using a laser](https://www.youtube.com/watch?v=q3Dn5U3cSWk)
5. [how to effeciently convert ROS PointCloud2 to pcl point cloud and visualize it in python](https://stackoverflow.com/questions/39772424/how-to-effeciently-convert-ros-pointcloud2-to-pcl-point-cloud-and-visualize-it-i?rq=1)
6. [Calculate X, Y, Z Real World Coordinates from Image Coordinates using OpenCV](https://www.fdxlabs.com/calculate-x-y-z-real-world-coordinates-from-a-single-camera-using-opencv/)