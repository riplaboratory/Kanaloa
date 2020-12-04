#!/usr/bin/env python


import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64


class CartJoyController:
    def __init__(self, wheel_topics, wheel_pub_range, steering_pub_topic=None):
        self.wheel_topics = wheel_topics
        self.wheel_pubs = []
        self.steering_pub = None
        self.joy_sub = rospy.Subscriber("joy", Joy, self.controller_callback)
        self.wheel_pub_range = wheel_pub_range

        self.init_wheel_pubs()

        if steering_pub_topic:
            self.steering_pub = rospy.Publisher(steering_pub_topic, Float64, queue_size=10)

    def init_wheel_pubs(self):
        for wheel in self.wheel_topics:
            # print(wheel)
            self.wheel_pubs.append(rospy.Publisher(wheel, Float64, queue_size=10))

    def controller_callback(self, joy_msg):
        # print(joy_msg.axes)
        throttle = 0.5 * (joy_msg.axes[5] - joy_msg.axes[2])
        steer_angle = joy_msg.axes[0]

        # print(throttle)

        mapped_value = (((throttle - -1) * (self.wheel_pub_range[1] - self.wheel_pub_range[0])) / (1 - -1)) + self.wheel_pub_range[0]

        # print(mapped_value)

        for pub in self.wheel_pubs:
            pub.publish(mapped_value)

        if self.steering_pub:
            self.steering_pub.publish(steer_angle)




if __name__ == '__main__':
    try:
        rospy.init_node('cart_joy_controller')

        cart_control = CartJoyController(["/my_robot_model/wheel_LF_joint_velocity_controller/command",
                                          "/my_robot_model/wheel_RF_joint_velocity_controller/command"],
                                          [-20, 20],
                                          "/my_robot_model/front_axle_joint_position_controller/command")

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
