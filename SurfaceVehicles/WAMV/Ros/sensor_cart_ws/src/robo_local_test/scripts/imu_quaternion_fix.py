#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu

from tf.transformations import euler_from_quaternion, quaternion_from_euler


class ImuOrientationFix:
    def __init__(self, sub_topic_name, pub_topic_name):
        pass
        self.fixed_imu_pub = rospy.Publisher(pub_topic_name, Imu, queue_size=4)
        self.imu_sub = rospy.Subscriber(sub_topic_name, Imu, self.callback)

    def callback(self, data):
        out_msg = data
        # print("Old MSG:")
        # print(out_msg)

        q_old = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]

        (r_old, p_old, y_old) = euler_from_quaternion(q_old)
        (r_new, p_new, y_new) = (y_old, p_old, r_old)

        q_new = quaternion_from_euler(r_new, p_new, y_new)

        out_msg.orientation.x = q_new[0]
        out_msg.orientation.y = q_new[1]
        out_msg.orientation.z = q_new[2]
        out_msg.orientation.w = q_new[3]

        self.fixed_imu_pub.publish(out_msg)

        # print("Old RPY:")
        # print(str(r_old) + " " + str(p_old) + " " + str(y_old))
        #
        # print("New RPY:")
        # print(str(r_new) + " " + str(p_new) + " " + str(y_new))
        #
        # print("New MSG:")
        # print(out_msg)


if __name__ == '__main__':

    rospy.init_node('imu_orientation_fix_node')

    imu_fix = ImuOrientationFix("/imu/data", "/imu/data/fixed")

    rospy.spin()
