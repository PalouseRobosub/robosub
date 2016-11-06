#!/usr/bin/env python

import rospy
from robosub.msg import visionPos as vision_pos
from robosub.msg import control


class Node():
    def __init__(self):
        self.pub = rospy.Publisher('control', control, queue_size=1)
        self.sub = rospy.Subscriber('vision/start_gate', vision_pos, self.callback)

    def callback(self, vision_result):
        msg = control()
        #check for empty message, we can't see a buoy
        if len(vision_result.xPos) is 0:
            #spin 10 degrees
            msg.yaw_state = control.STATE_RELATIVE
            msg.yaw_left = 10
            #don't move forward, maintain depth
            msg.forward_state = control.STATE_ERROR
            msg.forward = 0
            msg.dive_state = control.STATE_RELATIVE
            msg.dive = 0
        else: #we see a buoy, go towards it!
            msg.yaw_state = control.STATE_ERROR
            msg.yaw_left = 10 * vision_result.xPos[0]
            msg.dive_state = control.STATE_ERROR
            msg.dive = 10 * vision_result.yPos[0]

            #regulate distance
            msg.forward_state = control.STATE_ERROR
            error = vision_result.magnitude[0] - 10000
            msg.forward = error

        self.pub.publish(msg)





if __name__ == "__main__":
    rospy.init_node('jirwin_buoy_follower')
    node = Node()
    rospy.spin()
