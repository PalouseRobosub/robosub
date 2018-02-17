#!/usr/bin/env python

import rospy
from robosub_msgs.msg import visionPosArray
from robosub_msgs.msg import control


class Node():
    def __init__(self):
        self.pub = rospy.Publisher('control', control, queue_size=1)
        # we subscribe to the arbitrary "vision_topic" topic, when we use it
        # it is designed to be remapped to an actual vision topic
        self.sub = rospy.Subscriber('vision_topic', visionPosArray,
                                    self.callback)
        # state is just a string we print to the screen on every AI update so
        # we can see what the AI is thinking
        self.state = "BOOTUP"

    def callback(self, vision_result):
        msg = control()
        rospy.loginfo("state: {}".format(self.state))
        # check for empty message, we can't see a buoy
        if len(vision_result.data) == 0:
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
            msg.yaw_left = -50 * vision_result.data[0].xPos
            msg.dive_state = control.STATE_ERROR
            msg.dive = -1 * vision_result.data[0].yPos

            # regulate distance
            msg.forward_state = control.STATE_ERROR
            error = -50 * (vision_result.data[0].magnitude - 0.012)
            msg.forward = error

        self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node('basic_buoy_follower')
    node = Node()
    rospy.spin()
