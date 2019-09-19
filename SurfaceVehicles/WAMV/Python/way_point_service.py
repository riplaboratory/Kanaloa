#! /usr/bin/env python
import rospy                                      
from std_srvs.srv import Trigger, TriggerResponse

import way_point_class_WAMV as WAMV

wamv = WAMV()


def trigger_response(request):
    ''' 
    Callback function used by the service server to process
    requests from clients. It returns a TriggerResponse
    '''

    if request == ""


    return TriggerResponse(
        success=True,
        message="Hey, roger that; we'll be right there!"
    )

rospy.init_node('sos_service')                     # initialize a ROS node
my_service = rospy.Service(                        # create a service, specifying its name,
    '/fake_911', Trigger, trigger_response         # type, and callback
)


rospy.spin()  
