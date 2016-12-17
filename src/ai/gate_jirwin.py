#!/usr/bin/env python

import rospy
from robosub.msg import visionPosArray as vision_pos_array
from robosub.msg import control


class Node():
    def __init__(self):
        rospy.loginfo("Init done")
        self.pub = rospy.Publisher('control', control, queue_size=1)
        self.sub = rospy.Subscriber('vision/start_gate', vision_pos_array,
                                    self.callback)
        self.state = "SEARCHING"
        self.dir = "LEFT"

    def callback(self, vision_result):
        msg = control()

        if self.state is "SEARCHING":
            rospy.loginfo("state: {0} {1}".format(self.state, self.dir))
        else:
            rospy.loginfo("state: {}".format(self.state))

        # always maintain depth at 1 meter
        msg.dive_state = control.STATE_ABSOLUTE
        msg.dive = -1

        if self.state is "SEARCHING":
            if len(vision_result.data) > 1:  # if we can see the entire gate
                self.state = "TRACKING_GATE"
            else:  # can't see whole gate, spin
                # spin 10 degrees
                msg.yaw_state = control.STATE_RELATIVE
                if self.dir is "LEFT":
                    msg.yaw_left = 10
                else:
                    msg.yaw_left = -10
                # don't move forward
                msg.forward_state = control.STATE_ERROR
                msg.forward = 0
        elif self.state is "TRACKING_GATE":
            if len(vision_result.data) < 1:  # if we can no longer see the gate
                # set to blindly continue going forward for a bit
                self.exit_time = rospy.Time.now() + rospy.Duration.from_sec(10)
                self.state = "CONTINUE_GATE"
            elif len(vision_result.data) < 2:
                self.state = "SEARCHING"
                if self.dir is "LEFT":
                    self.dir = "RIGHT"
                else:
                    self.dir = "LEFT"
            else:
                # calculate center of gate,
                # which is the average of the two pole positions
                gate_center = (vision_result.data[0].xPos +
                               vision_result.data[1].xPos) / 2
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
                rospy.signal_shutdown()

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
