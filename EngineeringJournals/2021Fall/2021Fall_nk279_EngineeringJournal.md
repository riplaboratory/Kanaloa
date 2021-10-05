Kevin Nguyen
nk279@hawaii.edu
Team Kanaloa VIP

### Notes

11 September 2021
+ Brainstorming new project ideas
    + Putting ROS onto MRSUH
    + LiDAR for WAMV and possibly for any vessel (most likely doable)
        + Using Velodyne 16 LiDAR
        + sensor_msgs:PointCloud2 to pcl::PointCloud -> Turns from point cloud data to x, y, z
        + Sources:
            + [how to effeciently convert ROS PointCloud2 to pcl point cloud and visualize it in python](https://stackoverflow.com/questions/39772424/how-to-effeciently-convert-ros-pointcloud2-to-pcl-point-cloud-and-visualize-it-i?rq=1)
            + [[ROS Q&A] 040 - How to check the distance to an obstacle using a laser](https://www.youtube.com/watch?v=q3Dn5U3cSWk)
            + [What is the definition of the contents of PointCloud2?](https://robotics.stackexchange.com/questions/19290/what-is-the-definition-of-the-contents-of-pointcloud2)
            + [sensor_msgs/PointCloud2 Message](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html)
            + [sensor_msgs/PointField Message](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointField.html)

26 September 2021
+ Research into `sensor_msgs`
    + Uses [PointCloud2](http://wiki.ros.org/pcl/Overview) data:
        + The newly revised ROS point cloud message (and currently the de facto standard in PCL), now representing arbitrary n-D (n dimensional) data. Point values can now be of any primitive data types (int, float, double, etc), and the message can be specified as 'dense', with height and width values, giving the data a 2D structure, e.g. to correspond to an image of the same region in space.
    + It seems that ROS offers a C++ package (`pcl_ros`) and Python already has a pcl library.
    + [Building a Simple PCL Interface for Python](https://industrial-training-master.readthedocs.io/en/melodic/_source/session5/Simple-PCL-Interface-for-Python.html)

27 September 2021
+ Define a more solid project proposal
    + Draft: How can I use a sensor to detect objects so that the data can be later used for obstalce avoidance.
    + This would use a LiDAR to grab sensor_msgs from ROS
    + Comments from Kai J.: 
        + If the sensor your working with is the LiDAR, I think your problem is currently how to interpret data into something meaningful (for us)
        + If it’s for object detection, then maybe it’s to use sensor data to interpret “objects” which is an ambiguous definition

### Project
#### Description
> Object detection is the process of detectiong things that are not considered traversable terrain (water). The simplest variation of this for us to use in VRX would be to identify a vrx object from the camera using some type of classifier (such as a neural network). And then based off where we see the object in the picture, location the camera is mounted, location the lidar is mounted, we are able to read in the Lidar point cloud data (those red dots you see in my pictures) to try get an average of the distance in that particular area. 
>
> Then we can assume based on the WAMV's current posistion, and the distance we measured the object from the WAMV, we can then tell where the object is "globally", which is where the map part comes in.
>
> In terms of linearity of progression, first would come the object classifier (the neural network). You can also then have someone in parallel looking into how to read in Lidar data in regions we are interested in. The idea would be to then combine these two so when an object is detected in the neural network, you can then reference the point cloud to get the distance. The next step would then be to ensure that the detected object gets placed on the map (but for the most part, my mapping server will take that into account already, but some imporvments will need to be made).
>
> \- Raymond 

#### Mission Statement
To be able to use sensor data to interpret obstacles such as totems, polyforms, and surmarks and display it to the user so that information can be easily interpreted. This will be necessary in the future so that this data can be used for future problems such as behaviour planning. This will be a documented procedure so that any parts left unfinished can be easily transferred to a new member and further improve our understanding of the data.
#### Primary Objectives
Currently, our team does not have any existing code that is able to subscribe to `sensor_msgs` PointCloud2 data that is outputted and have no way to interpret it. What we have at the moment is the basic idea of how we will use object detection but have not methods implemented. This has halted our issues of being able to test and apply behaviour planning methods. Furthermore, to those who are not directly working with these messages, it can be easy to be misinterpret this ambiguous data set as the data is for more lower level programming (binary). Our objective is to be able to use a Point Cloud Library (PCL) in preferably Python to be able to interpret and output this data to the user. This piece of software will also be well documented with in-code comments and documentation so that it can be reworked in the future or easily implemented into other systems (WAM-V, UPSV, etc.).
#### Success Criteria
Success in object detection will be to have this basic idea of implemented in Python. It should be able to output data that is easily read and interpreted so that we can understand the `fields` object in the data set. This will also have documentation (`README.md` and in-code comments) that talks about the system set up (how to install ROS, what packages are needed), what each function does, as well as expected inputs and outputs. Implementation of this software should be able to be easily reconfigured and applied to different systems such as WAMV, UPSV, etc.
### Functional Requirements
| Functional Requirement ID | Objective |
| :---:                     |   :---   |
| <ul><li>[ ] 1</li></ul>   | Software is able to interepret the `fields` object in the `sensor_msgs` data set and output it so that a member can be able to read it |
| <ul><li>[ ] 2</li></ul>   | Documentation on created software to include system set-up, configuration, and expected output so that future members can be ready to modify it within a week | 
| <ul><li>[ ] 3</li></ul>   | Software will be easily adapatable in different systems Kanaloa uses |

### `sensor_msgs` key
+ fields.name - type `string`
+ fields.offset - type `uint32`
+ fields.datatype - type `uint8`
+ fields.count - type `uint32`
+ is_bigendian - is this data big endian?
+ point_step - length of point in bytes
+ row_step - length of row in bytes
+ data - actual point daya, size is (`row_step` * `height`)


