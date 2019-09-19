#! /usr/bin/env python
import rospy 
from way_point_wamv.srv import add_way_point, add_way_pointResponse


def way_point_response(lat, lon, minutes):
	print("way_point_response")

	print("LAT: " + str(lat))
	print("LON: " + str(lon))
	print("MINUTES: " + str(minutes))
	


	return add_way_pointResponse(recieved=True)


rospy.init_node('way_point_service')                     # initialize a ROS node

way_point_service = rospy.Service('way_point', add_way_point, way_point_response)

rospy.spin()
