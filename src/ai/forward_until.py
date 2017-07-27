#!/usr/bin/env python

import rospy
from rs_yolo.msg import DetectionArray as detection_array
from robosub.msg import control
from util import *

import sys

class ForwardUntilTask():

    def __init__(self, label_list):
        rospy.loginfo("Init done")
        self.pub = rospy.Publisher('control', control, queue_size=1)
        self.sub = rospy.Subscriber('vision', detection_array, self.callback)
        self.state = "SEARCHING"

        self.labels = label_list

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

    def callback(self, detection):
        detections = []
        for label_name in self.labels:
            detections.append(filterByLabel(detection.detections,
                                            label_name))

        vision_result = getMostProbable(detections, thresh=0.5)

        if vision_result is not None:
            rospy.logwarn("Found {}, stopping...".format([value.label for value in vision_result]))
            msg = control()
            msg.forward_state = control.STATE_ERROR
            msg.forward = 0
            rospy.signal_shutdown(0)
        

if __name__ == "__main__":
    rospy.init_node('forward_until_task')
    node = ForwardUntilTask(sys.argv[1])
    rospy.spin()
