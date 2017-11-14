#!/usr/bin/env python

import rospy
from rs_shared_msgs.msg import DetectionArray as detection_array
from robosub.msg import control
from util import *
class ForwardTask():

    def __init__(self):
        rospy.loginfo("Init done")
        self.pub = rospy.Publisher('control', control, queue_size=1)
        self.state = "SEARCHING"

        msg = control()
        # Maintain roll and pitch of 0
        msg.roll_state = control.STATE_ABSOLUTE
        msg.roll_right = 0
        msg.pitch_state = control.STATE_ABSOLUTE
        msg.pitch_down = 0
        # Don't move forward
        msg.forward_state = control.STATE_ERROR
        msg.forward = 1

        msg.dive_state = control.STATE_ABSOLUTE
        msg.dive = -1

        msg.yaw_state = control.STATE_RELATIVE
        msg.yaw_left = 0

        self.pub.publish(msg)

    def send(self):

        msg = control()
        # Maintain roll and pitch of 0
        msg.roll_state = control.STATE_ABSOLUTE
        msg.roll_right = 0
        msg.pitch_state = control.STATE_ABSOLUTE
        msg.pitch_down = 0
        # Don't move forward
        msg.forward_state = control.STATE_ERROR
        msg.forward = 1

        msg.dive_state = control.STATE_ABSOLUTE
        msg.dive = -1

        self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node('buoy_task')
    node = ForwardTask()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        node.send()
        rate.sleep()
