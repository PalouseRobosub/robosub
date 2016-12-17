#!/usr/bin/env python

import rospy
from robosub.msg import visionPosArray as vision_pos_array
from robosub.msg import control

class Node():
    
    def __init__(self):
        rospy.loginfo("Init done")
        self.pub = rospy.Publisher('control', control, queue_size=1)
        self.sub = rospy.Subscriber('vision/buoys/red', vision_pos_array, self.callback)
        self.state = "SEARCHING"

    def callback(self, vision_result):
        msg = control()

        #maintain roll and pitch of 0
        msg.roll_state = control.STATE_ABSOLUTE
        msg.roll_right = 0
        msg.pitch_state = control.STATE_ABSOLUTE
        msg.pitch_down = 0

        rospy.loginfo("state: {}".format(self.state))
        # check for empty message, we can't see a buoy
        if len(vision_result.data) < 1:
            self.state = "SEARCHING"
            # spin 10 degrees
            msg.yaw_state = control.STATE_RELATIVE
            msg.yaw_left = 10
            # don't move forward, maintain depth
            msg.forward_state = control.STATE_ERROR
            msg.forward = 0
            msg.dive_state = control.STATE_RELATIVE
            msg.dive = 0
        else:  # we see a buoy, go towards it!
            self.state = "TRACKING"
            msg.yaw_state = control.STATE_ERROR
            msg.yaw_left = vision_result.data[0].xPos / 10 * -1
            msg.dive_state = control.STATE_ERROR
            msg.dive = vision_result.data[0].yPos / 10 * -1

            # regulate distance
            msg.forward_state = control.STATE_ERROR
            error = (1 - vision_result.data[0].magnitude) * 1000
            msg.forward = error


        self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node('jirwin_buoy_follower')
    node = Node()
    rospy.spin()
