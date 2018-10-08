#!/usr/bin/env python

"""
comms.py

This is an ROS node that communicates uses the `comms_messenger` python library to communicate
with the Technical Director's info server for the 2018 RobotX Maritime challenge.
"""

import rospy
from comms_messenger import CommsMessenger

def main():
    # First operating should be node initialization.
    rospy.init_node('communication_protocol')
    rospy.loginfo('Creating node {} ...'.format(rospy.get_name()))

    # Get all parameters
    address = rospy.get_param('~address', '127.0.0.1')
    port = rospy.get_param('~port', 8080)
    team_id = rospy.get_param('~team_id', 'MANOA')

    rospy.loginfo('Parameters')
    rospy.loginfo('address: {}'.format(address))
    rospy.loginfo('port   : {}'.format(port))
    rospy.loginfo('team_id: {}'.format(team_id))

    # Instances
    rate = rospy.Rate(hz=1)
    comms_msgr = CommsMessenger(address, port, team_id)

    while not rospy.is_shutdown():
        # Send heartbeat message every second.
        # TODO: Collect all fields from ROS and use those instead of hardcoded values.
        heartbeat_fields = {
            'latitude': 21.31198,
            'north_south_indicator': 'N',
            'longitude': 157.88972,
            'east_west_indicator': 'W',
            'system_mode': 2,
            'auv_status': 1
        }
        try:
            results = comms_msgr.send_heartbeat(**heartbeat_fields)
            message = results['message'] 
            rospy.loginfo('sent message: {}'.format(message))
        except RuntimeError as err:
            rospy.logerr('Exception while sending heartbeat: {}'.format(err))

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass