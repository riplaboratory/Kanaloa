#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix

class GPSFrameFix:

    def __init__(self, sub_topic_name, frame_id):
        self.frame_id = frame_id
        self.fixed_gps_pub = rospy.Publisher("/fix/fixed", NavSatFix, queue_size=10)
        self.gps_subscriber = rospy.Subscriber(sub_topic_name, NavSatFix, self.callback)

    def callback(self, data):
        # out_msg = NavSatFix
        out_msg = data
        out_msg.header.frame_id = self.frame_id

        self.fixed_gps_pub.publish(out_msg)


if __name__ == '__main__':

    rospy.init_node('gps_frame_fix_node', anonymous=True)

    gps_ff = GPSFrameFix("/fix", "gps")

    rospy.spin()
