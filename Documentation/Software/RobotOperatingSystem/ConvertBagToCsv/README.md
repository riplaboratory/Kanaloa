# Converting a ROS .bag file to a .csv

This is a quick tutorial on the best method (in our opinion) of converting a ros .bag file to a .csv or .txt file.

This theoretically works for any topic in a .bag file; however, for topics containing large amounts of information (like image topics should probably be avoided.

 1. Navigate to the directory with your .bag file in terminal
 2. type `rostopic echo -b [bag file name].bag -p [topic name] > output.csv`
 
You can only output one topic at a time--but for most cases, this is probably preferrable behavior, as the topics will be individually time stamped.  Run this line multiple times for different topics (make sure you change the name of the `output.txt` file, or else it will be overwritten.  You can record all topics, replace `[topic name]` with `-a`, but this does result in an extremely messy output file that will need to be sorted in post processing if you want to extract individual topics.  

You can also output a .txt file by changing the extension of `output.csv` to `output.txt`.

## Other methods

Other methods exist, but require more work for the same result, but if you are looking for some typie of specific functionality (e.g. batch export) then parhaps they will work better for you.  Some options to try are: 

 -[rosbag_to_csv](https://github.com/AtsushiSakai/rosbag_to_csv)
 -[ClearPath Robotics CONVERTING ROS BAG TO CSV](https://www.clearpathrobotics.com/assets/guides/ros/Converting%20ROS%20bag%20to%20CSV.html)
