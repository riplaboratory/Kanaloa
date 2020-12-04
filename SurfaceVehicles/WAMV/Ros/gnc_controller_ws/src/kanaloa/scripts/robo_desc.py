#!/usr/bin/env python

import rospy

wamv_robo_desc = rospy.get_param("/wamv/robot_description")

rospy.set_param("/robot_description", wamv_robo_desc)
