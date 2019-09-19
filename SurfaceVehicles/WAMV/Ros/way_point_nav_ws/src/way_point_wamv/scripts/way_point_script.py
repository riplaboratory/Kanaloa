#! /usr/bin/env python
import rospy 
from way_point_wamv.srv import *

# print(dir(add_way_point))
# print("---")
# print(dir(add_way_point._request_class))
# print("***")
# print(dir(add_way_point._response_class))


def way_point_response_func(req):

	lat = req.latitude
	lon = req.longitude
	minutes = req.minutes

	print("way_point_response_func")

	print("LAT: " + str(lat))
	print("LON: " + str(lon))
	print("MINUTES: " + str(minutes))
	


	return add_way_pointResponse(recieved=True)


rospy.init_node('way_point_service')                     # initialize a ROS node

way_point_service = rospy.Service('way_point', add_way_point, way_point_response_func)

rospy.spin()
