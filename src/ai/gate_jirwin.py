#!/usr/bin/env python

import rospy
from robosub.msg import visionPos as vision_pos
from robosub.msg import control


class Node():
    def __init__(self):
        self.pub = rospy.Publisher('control', control, queue_size=1)
        self.sub = rospy.Subscriber('vision/buoy', vision_pos, self.callback)
        self.state = "SEARCHING"

    def callback(self, vision_result):
        msg = control()

        #always maintain depth at 1 meter
        msg.dive_state = control.STATE_ABSOLUTE
        msg.dive = -1

        if self.state is "SEARCHING":
            if len(vision_result.xPos) > 1: #if we can see the entire gate
                self.state = "TRACKING_GATE"
            else: #can't see whole gate, spin
                #spin 10 degrees
                msg.yaw_state = control.STATE_RELATIVE
                msg.yaw_left = 10
                #don't move forward
                msg.forward_state = control.STATE_ERROR
                msg.forward = 0
        elif self.state is "TRACKING_GATE":
            if len(vision_result.xPos) < 1: #if we can no longer see the gate
                self.exit_time = rospy.Time.now() + rospy.Duration.from_sec(10)
                self.state = "CONTINUE_GATE"

            gate_center = (vision_result.xPos[0] + vision_result.xPos[1]) / 2
            msg.yaw_state = control.STATE_ERROR
            msg.yaw_left = 10 * gate_center

            #regulate distance
            msg.forward_state = control.STATE_ERROR
            msg.forward = 10
        elif self.state is "CONTINUE_GATE":
            #check if we've gone straight for long enough
            if rospy.Time.now() > self.exit_time:
                rospy.shutdown()

            #maintain heading
            msg.yaw_state = control.STATE_RELATIVE
            msg.yaw_left = 0

            #go straight
            msg.forward_state = control.STATE_ERROR
            msg.forward = 10


        self.pub.publish(msg)





if __name__ == "__main__":
    rospy.init_node('jirwin_buoy_follower')
    node = Node()
    rospy.spin()
