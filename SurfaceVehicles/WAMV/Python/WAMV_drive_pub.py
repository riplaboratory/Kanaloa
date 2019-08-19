#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32

import sys

def talker(pub_names, pub_values):
    publishers = []
    for pub_name in pub_names:
        publishers.append(rospy.Publisher(pub_name, Int32, queue_size=10))

    rospy.init_node('wamv_thrust_publisher_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        for i in range(len(pub_values)):
            publishers[i].publish(pub_values[i])
            # pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':

    command = raw_input("\n\t What direction do you want to go (f, b, s, l, r): \n\t")

    if command != "s":
        speed = int(raw_input("\n\t What speed do you want to go (0 to 1000): \n\t"))


    try:

        if command == "f":
            print("\n\t\t Going Forward")
            talker(["q3_thruster_input", "q4_thruster_input"], [speed, speed])
            # talker(["q3_thruster_input", "q4_thruster_input"], [900, 900])
            
        elif command == "b":
            print("\n\t\t Going Backward")
            talker(["q3_thruster_input", "q4_thruster_input"], [-speed, -speed])
            # talker(["q3_thruster_input", "q4_thruster_input"], [900, 900])
            
        elif command == "l":
            print("\n\t\t Turning Left")
            talker(["q3_thruster_input", "q4_thruster_input"], [-speed, speed])
            # talker(["q3_thruster_input", "q4_thruster_input"], [900, 900])
            
        elif command =="r":
            print("\n\t\t Turning Right")
            talker(["q3_thruster_input", "q4_thruster_input"], [speed, -speed])
            # talker(["q3_thruster_input", "q4_thruster_input"], [900, 900])
            
        elif command =="s":
            print("\n\t\t Stopping")
            talker(["q3_thruster_input", "q4_thruster_input"], [0,0])
            
        else:
            sys.exit("Invalid Command")

    except rospy.ROSInterruptException:
        pass