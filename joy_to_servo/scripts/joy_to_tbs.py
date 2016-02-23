#!/usr/bin/env python

# Copyright (C) 2014, Massachusetts Institute of Technology
# All rights reserved.
#
# This work is sponsored by the Department of the Air Force Air Force
# contract number: FA8721-05-C-0002. The opinions, interpretations,
# recommendations, and conclusions are those of the author and are
# not necessarily endorsed by the United Stated Government.

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from racecar.msg import ThrottleBrakeSteering

class joy_to_tsb:
    def __init__(self):

        # get parameters
        self.throttle_axis = rospy.get_param("~throttle_axis", 1)
        self.throttle_positive_gain = rospy.get_param("~throttle_positive_gain", 1)
        self.throttle_negative_gain = rospy.get_param("~throttle_negative_gain", 1)
        self.steering_axis = rospy.get_param("~steering_axis", 1)
        self.steering_positive_gain = rospy.get_param("~steering_positive_gain", 1)
        self.steering_negative_gain = rospy.get_param("~steering_negative_gain", 1)
        self.brake_axis = rospy.get_param("~brake_axis", 1)
        self.brake_positive_gain = rospy.get_param("~brake_positive_gain", 1)

        # todo: check parameters

        # advertise servo commands
        self.pub = rospy.Publisher("/car_commands", ThrottleBrakeSteering, queue_size=10)

        # subscribe to joy topic
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joyCallback)
		
    def joyCallback(self, joy_msg):
        # get joystick throttle value
        if self.throttle_axis >= len(joy_msg.axes):
            rospy.logerr("Throttle axis index (%s) greater than Joy message axes length (%s), ignoring.",
                         self.throttle_axis, len(joy_msg.axes))
            return
        throttle = joy_msg.axes[self.throttle_axis]
        if throttle > 1 or throttle < -1:
            rospy.logerr("Joy throttle axis value (%s) outside of expected range, ignoring.", throttle)
            return

        # get joystick steering value
        if self.steering_axis >= len(joy_msg.axes):
            rospy.logerr("Steering axis index (%s) greater than Joy message axes length (%s), ignoring.",
                         self.steering_axis, len(joy_msg.axes))
            return
        steering = joy_msg.axes[self.steering_axis]
        if steering > 1 or steering < -1:
            rospy.logerr("Joy steering axis value (%s) outside of expected range, ignoring.", steering)
            return

        # get joystick brake value
        if self.brake_axis >= len(joy_msg.axes):
            rospy.logerr("Brake axis index (%s) greater than Joy message axes length (%s), ignoring.",
                         self.brake_axis, len(joy_msg.axes))
            return
        brake = joy_msg.axes[self.brake_axis]
        if brake > 1 or brake < -1:
            rospy.logerr("Joy brake axis value (%s) outside of expected range, ignoring.", steering)
            return

        # todo; trim axes?

        # apply gains
        if throttle > 0:
            throttle = throttle * self.throttle_positive_gain
        elif throttle < 0:
            throttle = throttle * self.throttle_negative_gain

        if steering > 0:
            steering = steering * self.steering_positive_gain
        elif steering < 0:
            steering = steering * self.steering_negative_gain

        if brake > 0:
            brake = brake * self.brake_positive_gain
        elif brake < 0:
            brake = 0

        # limit commands
        if throttle > 1:
            throttle = 1
        elif throttle < -1:
            thottle = -1

        if steering > 1:
            steering = 1
        elif steering < -1:
            steering = -1

        if brake > 1:
            brake = 1

	msg = ThrottleBrakeSteering()
	msg.throttle = throttle
	msg.steering = steering
	msg.brake = brake
        # publish servo commands
        self.pub.publish(msg)
		
if __name__ == '__main__':
    rospy.init_node('joy_to_tsb')
    node = joy_to_tsb()
    rospy.spin()
