#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
import sys
# from robotx_gazebo import UsvDrive

# def talker():
#     pub = rospy.Publisher('chatter', String, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()

# def boat_control():
#     pub = rospy.Publisher('cmd_drive', robotx_gazebo/UsvDrive, queue_size=10)
#     rospy.init_node('boat_script', anonymous=True)
#     rate = rospy.Rate(0.1) # 10hz
#     while not rospy.is_shutdown():
#         command_string = '{left: 1.0, right: 1.0}'
#         rospy.loginfo(command_string)
#         pub.publish(command_string)
#         rate.sleep()

left_front_pub = rospy.Publisher('left_front_thrust_cmd', Float32, queue_size=10)
left_rear_pub = rospy.Publisher('left_rear_thrust_cmd', Float32, queue_size=10)
right_front_pub = rospy.Publisher('right_front_thrust_cmd', Float32, queue_size=10)
right_rear_pub = rospy.Publisher('right_rear_thrust_cmd', Float32, queue_size=10)

def forward():
    left_front_pub.publish(1.0)
    left_rear_pub.publish(1.0)
    right_front_pub.publish(1.0)
    right_rear_pub.publish(1.0)


def reverse():
    left_front_pub.publish(-1.0)
    left_rear_pub.publish(-1.0)
    right_front_pub.publish(-1.0)
    right_rear_pub.publish(-1.0)

def right():
    left_front_pub.publish(0.5)
    left_rear_pub.publish(-0.5)
    right_front_pub.publish(-0.5)
    right_rear_pub.publish(0.5)

def left():
    left_front_pub.publish(-0.5)
    left_rear_pub.publish(0.5)
    right_front_pub.publish(0.5)
    right_rear_pub.publish(-0.5)

def clockwise():
    left_front_pub.publish(0.5)
    left_rear_pub.publish(0.5)
    right_front_pub.publish(-0.5)
    right_rear_pub.publish(-0.5)

def counter_clockwise():
    left_front_pub.publish(-0.5)
    left_rear_pub.publish(-0.5)
    right_front_pub.publish(0.5)
    right_rear_pub.publish(0.5)


if __name__ == '__main__':
    try:
        rospy.init_node('boat_script', anonymous=True)
        rate = rospy.Rate(0.1) # 10hz
        print("Please type the following keys (and then enter):\n w: forward \n a: strafe left \n s: reverse \n d: right \n q: counter_clockwise \n e: clockwise")
        while not rospy.is_shutdown():
            text = raw_input("")  # Python 2
            # print(text)
            if text == "a":
                left()
            elif text == "d":
                right()
            elif text == "w":
                forward()
            elif text == "s":
                reverse()
            elif text == "q":
                counter_clockwise()
            elif text == "e":
                clockwise()
            else:
                print("Invalid command")
            # print("Hello World")
            # command_string = '{left: 1.0, right: 1.0}'
            # rospy.loginfo(command_string)
            # pub.publish(command_string)
            # rate.sleep()

    except rospy.ROSInterruptException:
        pass