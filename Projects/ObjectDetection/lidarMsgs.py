#!/usr/bin/env python

""" 
  Name: lidarMsgs.py
  Created by: Team Kanaloa (http://rip.eng.hawaii.edu/research/unmanned-x-systems/)
  LICENSE: Internal Kanaloa use only
    
  Description: Program to print out messages from /wamv/sensors/lidars/lidar_wamv/points
               It is meant to be an example usage of how to access the field's x, y, z
               coordinates so that it can be used for other scripts in the future
  # INPUTS - N/A
  # OUTPUTS
      ## Output 1: "x: %f\ty: %f\tz: %f"
      ## This is an example of what the code interprets from the Lidar's fields x, y, z
  
  # Version History
    @date     2021.11.29 
    @author   Kevin Nguyen (nk279@hawaii.edu)
    @brief    Initial creation
"""

import rospy
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2

class Lidar():
    def printLidar(data):
        """Main function that prints outputs the distance to
        the object from LiDAR
        Output
        ------
        "x: %f\ty: %f\tz: %f"
        """
        # Iterate through pointcloud2 data and pull that x, y, z fields from it
        for p in pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True):
            print("x: %f\ty: %f\tz: %f" % (p[0],p[1],p[2]))

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
