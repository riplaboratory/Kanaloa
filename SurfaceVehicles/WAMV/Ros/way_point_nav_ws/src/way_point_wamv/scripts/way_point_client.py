#!/usr/bin/env python

import sys
import rospy
# from beginner_tutorials.srv import *
from way_point_wamv.srv import add_way_point, add_way_pointResponse


def way_point_client(lat, lon, minutes):
    rospy.wait_for_service('way_point')
    try:
        client_func = rospy.ServiceProxy('way_point', add_way_point)
        resp1 = client_func(lat, lon, minutes)
        return resp1.recieved
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [lat lon minutes]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 4:
        lat = float(sys.argv[1])
        lon = float(sys.argv[2])
        minutes = float(sys.argv[3])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s+%s+%s"%(lat, lon, minutes)
    print "%s + %s  + %s = %s"%(lat, lon, minutes, way_point_client(lat, lon, minutes))