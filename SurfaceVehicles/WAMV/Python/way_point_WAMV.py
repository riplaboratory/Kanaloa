#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu

from math import radians
from math import atan2
from math import cos
from math import sin
from math import acos
from math import asin
from math import degrees
from math import sqrt

import sys
import time

#Change base off IMU performance
imu_degree_offset_bias = 0


publishers = []
subscribers = []

desired_coordinates = []
current_coordinates = []
degree_offset = 0
distance = 0

def create_publisher_object(pub_name, pub_type):
    rospy_publisher = rospy.Publisher(pub_name, pub_type, queue_size=10)
    publisher_object = {"topic": pub_name, "message": pub_type, "publisher": rospy_publisher, "node_type":"Publisher"}
    return publisher_object

def create_subscriber_object(sub_name, sub_type, callback):
    rospy_subscriber = rospy.Subscriber(sub_name, sub_type, callback)
    subscriber_object = {"topic": sub_name, "message": sub_type, "subscriber": rospy_subscriber, "node_type":"Subscriber"}
    return subscriber_object


def navigation(desired_gps_coords, current_gps_coords, current_heading_degrees):
    #convert to coodinates to radians
    lat1,lon1,lat2,lon2 = map(radians, [lat1,lon1,lat2,lon2])

    # Calculate distance between GPS coordinates (Haversine Formula)
    deltalon = lon2 - lon1
    deltalat = lat2 - lat1
    alpha = 2*asin( sqrt( (sin(abs(deltalat)/2))**2 + cos(lat1)*cos(lat2)*((sin(abs(deltalon)/2))**2) ) )
    distance = alpha * r

    #print('\nDistance = ', distance, 'km')

    #OKAY NOW turn angle calc
    x = cos(lat2)*sin(deltalon)
    y = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(deltalon)
    bearing = degrees(atan2(y,x)) #Measured from East (-180,180) ccw+ cw-

    #print('\nDEBUG initbearing = ',bearing)

    #Want bearing to be measured ccw from East (0,360)
    if bearing<0 and bearing>-180:
        bearing += 360

def calculate_distance(current_gps_coords, desired_gps_coords):
    r = 6371 #radius of Earth, km

    lat1_in, lon1_in = current_gps_coords
    lat2_in, lon2_in = desired_gps_coords

    #convert to coodinates to radians
    lat1,lon1,lat2,lon2 = map(radians, [lat1_in,lon1_in,lat2_in,lon2_in])

    # Calculate distance between GPS coordinates (Haversine Formula)
    deltalon = lon2 - lon1
    deltalat = lat2 - lat1
    alpha = 2*asin( sqrt( (sin(abs(deltalat)/2))**2 + cos(lat1)*cos(lat2)*((sin(abs(deltalon)/2))**2) ) )
    distance = alpha * r * 1000 # Distance in meters

    return distance

def caclulate_rotation(current_gps_coords, desired_gps_coords, heading):
    lat1, lon1 = current_gps_coords
    lat2, lon2 = desired_gps_coords

    deltalon = lon2 - lon1
    deltalat = lat2 - lat1

    #OKAY NOW turn angle calc
    x = cos(lat2)*sin(deltalon)
    y = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(deltalon)
    bearing = degrees(atan2(y,x)) #Measured from East (-180,180) ccw+ cw-

    #print('\nDEBUG initbearing = ',bearing)

    #Want bearing to be measured ccw from East (0,360)
    # if bearing<0 and bearing>-180:
    #     bearing += 360

    theta = heading - bearing #theta is going to be turn angle

    return theta


def distance_manager(gps_data):
    global desired_coordinates
    global current_coordinates
    global publishers
    global degree_offset
    global distance


    # print("distance_manager")

    current_coordinates = [gps_data.latitude, gps_data.longitude]

    # lat = gps_data.latitude
    # lon = gps_data.longitude

    distance = calculate_distance([gps_data.latitude, gps_data.longitude], desired_coordinates) #Distance in meters

    # publishers[""]
    q3 = next(item for item in publishers if item["topic"] == "q3_thruster_input")
    q4 = next(item for item in publishers if item["topic"] == "q4_thruster_input")



    if degree_offset > 30:
        #Left and right pins are switched on arduino
        q3["publisher"].publish(-750)
        q4["publisher"].publish(750)
    elif degree_offset < -30:
        q3["publisher"].publish(750)
        q4["publisher"].publish(-750)

    elif distance > 20:
        left_thrust = 750 - degree_offset / 2
        right_thrust = 750 + degree_offset / 2

        if left_thrust >= 900: left_thrust = 900
        if left_thrust <= -900: left_thrust = -900
        if right_thrust >= 900: right_thrust = 900
        if right_thrust <= -900: right_thrust = -900

        q3["publisher"].publish(left_thrust)
        q4["publisher"].publish(right_thrust)
    elif distance > 10:
        q3["publisher"].publish(400 - degree_offset / 2)
        q4["publisher"].publish(400 + degree_offset / 2)
    elif distance <= 10:
        q3["publisher"].publish(0)
        q4["publisher"].publish(0)



def rotation_manager(imu_data):
    global current_coordinates
    global desired_coordinates
    global degree_offset

    # if len(current_coordinates) < 2

    # print("rotation_manager")

    q = imu_data.orientation

    yaw_rad = atan2(2.0 * (q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)
    yaw_deg = (yaw_rad+3.14159/2)*-180/3.14159
    yaw_deg -= imu_degree_offset_bias # YAW in terms of north, gets larger when going cw

    # print("yaw_deg: " + str(yaw_deg))

    yaw_deg += 90
    yaw_deg *= -1

    # print("yaw_deg post_process")

    theta_offset = caclulate_rotation(current_coordinates, desired_coordinates, yaw_deg)
    # theta_offset += imu_degree_offset_bias
    # theta_offset *= -1

    if theta_offset > 180:
        theta_offset -= 360
    elif theta_offset < -180:
        theta_offset += 360
    # print(theta_offset)

    if -5 < theta_offset < 5:
        degree_offset = 0
        # print("no degree offset")
    else:
        degree_offset = theta_offset
        # print(degree_offset)




    "finish this"





def wamv_navigation():
    publishers = []
    for pub_name in pub_names:
        publishers.append(rospy.Publisher(pub_name, Int32, queue_size=10))

    rospy.init_node('wamv_navigation_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        for i in range(len(pub_values)):
            publishers[i].publish(pub_values[i])
            # pub.publish(hello_str)
        rate.sleep()

def wamv_navigator():
    global publishers
    global subscribers
    global current_coordinates
    global desired_coordinates
    global degree_offset

    rospy.init_node('wamv_navigation_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz


    publishers.append(create_publisher_object("q3_thruster_input", Int32))
    publishers.append(create_publisher_object("q4_thruster_input", Int32))

    subscribers.append(create_subscriber_object("fix", NavSatFix, distance_manager))
    subscribers.append(create_subscriber_object("/imu/data", Imu, rotation_manager))

    while not rospy.is_shutdown():
        print("Current Coordinates: " + str(current_coordinates))
        print("Desired Coordinates: " + str(desired_coordinates))
        print("Degree Offset: " + str(degree_offset))
        print("Distance: " + str(distance))
        print("\n\n ---------- \n\n")
        time.sleep(0.5)


if __name__ == '__main__':
    
    global desired_coordinates

    lat2 = float(input('Enter desired latitude (N+ S-):\t\t'))
    lon2 = float(input('Enter desired longitude (E+ W-):\t'))

    desired_coordinates = [lat2, lon2]

    try:
        wamv_navigator()
    except rospy.ROSInterruptException:
        pass