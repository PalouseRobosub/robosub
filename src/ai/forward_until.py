#!/usr/bin/env python

import rospy
from rs_yolo.msg import DetectionArray as detection_array
from robosub.msg import control
from util import *

import sys

class ForwardUntilTask():

    def __init__(self, label_list, yaw_value):
        rospy.loginfo("Init done")
        self.pub = rospy.Publisher('control', control, queue_size=1)
        self.sub = rospy.Subscriber('vision', detection_array, self.callback)

        self.labels = label_list
        self.yaw_value = float(yaw_value)

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

        msg.yaw_state = control.STATE_ABSOLUTE
        msg.yaw_left = self.yaw_value

        self.pub.publish(msg)

        rospy.loginfo("Proceeding forward until {} label(s) are found".format(self.labels))

    def callback(self, detection):
        detections = []
        for label_name in self.labels:
            detections[-1:] = filterByLabel(detection.detections,
                                            label_name)

        rospy.loginfo("Found: {}".format([detection.label + "\n" for detection in detections]))

        vision_result = getMostProbable(detections, thresh=0.5)

        if vision_result is not None:
            rospy.loginfo("Seeing: {}".format(vision_result.label))

        msg = control()

        if vision_result is not None:
            rospy.logwarn("Found {}, stopping...".format(vision_result.label))
            msg.forward_state = control.STATE_ERROR
            msg.forward = 0
            self.pub.publish(msg)
            rospy.signal_shutdown(0)
        else:
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

            msg.yaw_state = control.STATE_ABSOLUTE
            msg.yaw_left = self.yaw_value

        self.pub.publish(msg)



if __name__ == "__main__":
    rospy.init_node('forward_until_task')

    label_list = []
    for string in sys.argv[2:]:
        if string[0] is not '_':
            label_list.append(string)
    node = ForwardUntilTask(label_list, sys.argv[1])
    rospy.spin()
