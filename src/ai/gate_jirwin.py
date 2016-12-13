#!/usr/bin/env python

import rospy
from robosub.msg import visionPos as vision_pos
from robosub.msg import control


class Node():
    def __init__(self):
        self.pub = rospy.Publisher('control', control, queue_size=1)
        self.sub = rospy.Subscriber('vision/start_gate', vision_pos,
                                    self.callback)
        self.state = "SEARCHING"

    def callback(self, vision_result):
        msg = control()

        rospy.info("state: {}".format(self.state))

        # always maintain depth at 1 meter
        msg.dive_state = control.STATE_ABSOLUTE
        msg.dive = -1

        if self.state is "SEARCHING":
            if len(vision_result.xPos) > 1:  # if we can see the entire gate
                self.state = "TRACKING_GATE"
            else:  # can't see whole gate, spin
                # spin 10 degrees
                msg.yaw_state = control.STATE_RELATIVE
                msg.yaw_left = 10
                # don't move forward
                msg.forward_state = control.STATE_ERROR
                msg.forward = 0
        elif self.state is "TRACKING_GATE":
            if len(vision_result.xPos) < 1:  # if we can no longer see the gate
                # set to blindly continue going forward for a bit
                self.exit_time = rospy.Time.now() + rospy.Duration.from_sec(10)
                self.state = "CONTINUE_GATE"

            # calculate center of gate,
            # which is the average of the two pole positions
            gate_center = (vision_result.xPos[0] + vision_result.xPos[1]) / 2
            msg.yaw_state = control.STATE_ERROR
            msg.yaw_left = 10 * gate_center

            # go forward
            msg.forward_state = control.STATE_ERROR
            msg.forward = 10

        # at this point, we should be blindly going forward for a bit to pass
        # through the gate
        elif self.state is "CONTINUE_GATE":
            # check if we've gone straight for long enough
            if rospy.Time.now() > self.exit_time:
                rospy.shutdown()

            # maintain heading
            msg.yaw_state = control.STATE_RELATIVE
            msg.yaw_left = 0

            # go straight
            msg.forward_state = control.STATE_ERROR
            msg.forward = 10
        else:
            rospy.error("got into uncaught state: {}".format(self.state))


        self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node('jirwin_gate_task')
    node = Node()
    rospy.spin()
