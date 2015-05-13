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
from racecar.msg import EmergencyStop

class joy_to_emergency_stop:
    def __init__(self):
        # get parameters
        self.estop_button = rospy.get_param("~estop_button", 1)
        self.release_estop_button = rospy.get_param("~release_estop_button", 1)
        # advertise servo commands
        self.estop_pub = rospy.Publisher("/emergencyStop", EmergencyStop, queue_size=10)
        # subscribe to joy topic
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joyCallback)

    def joyCallback(self, joy_msg):
	if joy_msg.buttons[self.estop_button]:
            msg = EmergencyStop()
            msg.estop=True;
            msg.values.append(0);
            msg.values.append(0);
            self.estop_pub.publish(msg)
            rospy.logwarn("Emergency stop from joystick")
	if joy_msg.buttons[self.release_estop_button]:
            msg = EmergencyStop()
            msg.estop=False;
            self.estop_pub.publish(msg)
            rospy.logwarn("Emergency stop released from joystick")

if __name__ == '__main__':
    rospy.init_node('joy_to_servo')
    node = joy_to_emergency_stop()
    rospy.spin()
