#!/usr/bin/env python

import sys
import getopt
import rospy
from robosub.msg import visionPosArray as vision_pos_array
from robosub.msg import control


class GateTask():
    def __init__(self, commandArgs):
        rospy.loginfo("Init done")

        #Set blind forward duration default to 1 second
        self.duration = 1

        #Use command line arguments to get duration
        try:
            opts, args = getopt.getopt(commandArgs, "hd:", ["duration="])
        except getopt.GetoptError:
            print "Usage: gate_jirwin.py -d <duration>"
            sys.exit(2)
        for opt, arg in opts:
            if opt == '-h':
                #Print usage message
                print "Usage: gate_jirwin.py -d <duration>"
                print "       or"
                print "       gate_jirwin.py --duration=<duration>"
                sys.exit()
            elif opt in ("-d", "--duration"):
                #Fetch duration
                self.duration = int(arg)
                rospy.loginfo("Setting duration to {} seconds.".format(
                              self.duration))

        self.pub = rospy.Publisher('control', control, queue_size=1)
        self.sub = rospy.Subscriber('vision/start_gate', vision_pos_array,
                                    self.callback)
        self.state = "SEARCHING_LEFT"
        #How far off center the gate should be
        self.errorGoal = 0.1
        #Used to determine if moving we were moving left or right last
        self.prev_yaw = 0
        self.completeTime = None

    def noneFound(self, msg, vision):
        msg.yaw_state = control.STATE_ERROR
        msg.forward_state = control.STATE_ERROR
        msg.forward = 0
        msg.dive_state = control.STATE_ABSOLUTE
        msg.dive = -2

        if self.state is "LOST":
            msg.yaw_left = 10
        elif self.state is "SEARCHING LEFT":
            self.state = "SEARCHING RIGHT"
            msg.yaw_left = -10
        elif self.state is "SEARCHING RIGHT":
            self.state = "SEARCHING LEFT"
            msg.yaw_left = 10
        elif self.state is "TRACKING":
            self.state = "LOST"
            msg.yaw_left = 0
        else:
            # Complete
            pass

        return msg

    def oneFound(self, msg, vision):
        if self.state is "LOST":
            if vision.data[0].xPos < 0:
                self.state = "SEARCHING_RIGHT"
                msg.yaw_left = -10
            else:
                self.state = "SEARCHING_LEFT"
                msg.yaw_left = 10
        elif self.state is "SEARCHING_LEFT" or self.state is "SEARCHING_RIGHT":
            pass
        elif self.state is "TRACKING":
            msg.forward_state = control.STATE_ERROR
            msg.forward = 0
            msg.yaw_state = control.STATE_ERROR
            msg.yaw_left = self.prev_yaw * -1
            if msg.yaw_left > 0:
                self.state = "SEARCHING_LEFT"
            elif msg.yaw_left < 0:
                self.state = "SEARCHING_RIGHT"
            else:
                if vision.data[0].xPos < 0:
                    self.state = "SEARCHING_RIGHT"
                    msg.yaw_left = 10
                else:
                    self.state = "SEARCHING_LEFT"
                    msg.yaw_left = -10

        return msg

    def twoFound(self, msg, vision):
        self.state = "TRACKING"

        gateXPos = (vision.data[0].xPos + vision.data[1].xPos) / 2
        gateYPos = (vision.data[0].yPos + vision.data[1].yPos) / 2

        if abs(gateXPos) > self.errorGoal:
            msg.yaw_state = control.STATE_RELATIVE
            msg.yaw_left = gateXPos * -50
            rospy.loginfo("Yaw output: {}".format(msg.yaw_left))
        elif abs(gateYPos) > self.errorGoal:
            msg.dive_state = control.STATE_RELATIVE
            msg.dive = gateYPos * -50
            rospy.loginfo("Dive output: {}".format(msg.dive))
        else:
            if (vision.data[0].magnitude + vision.data[1].magnitude) > 0.012:
                self.state = "COMPLETE"
            else:
                msg.yaw_state = control.STATE_RELATIVE
                msg.yaw_left = 0
                msg.dive_state = control.STATE_RELATIVE
                msg.dive = 0
                msg.forward_state = control.STATE_ERROR
                msg.forward = 10
        return msg


    def callback(self, vision_result):
        performTask = {0: self.noneFound,
                       1: self.oneFound,
                       2: self.twoFound,
                       }

        msg = control()

        msg.roll_state = control.STATE_ABSOLUTE
        msg.roll_right = 0
        msg.pitch_state = control.STATE_ABSOLUTE
        msg.pitch_down = 0

        rospy.loginfo("state: {}".format(self.state))

        if self.state is not "COMPLETE":
            numFound = len(vision_result.data)
            msg = performTask[numFound](msg, vision_result)
        elif not self.completeTime:
                self.completeTime = rospy.get_rostime()
                rospy.loginfo("Complete time: {}".format(self.completeTime))

        # always maintain depth at 1 meter
        msg.dive_state = control.STATE_ABSOLUTE
        msg.dive = -1

        if self.state is "COMPLETE" and self.completeTime is not None:
            msg.forward_state = control.STATE_ERROR
            if self.completeTime + rospy.Duration(self.duration) <\
               rospy.get_rostime():
                msg.forward = 0
                rospy.loginfo("Complete")
                rospy.signal_shutdown(0)
            else:
                rospy.loginfo("Moving through gate")
                msg.forward = 10
            msg.yaw_state = control.STATE_RELATIVE
            msg.yaw_left = 0
            msg.dive_state = control.STATE_RELATIVE
            msg.dive = 0

        self.prev_yaw = msg.yaw_left
        self.pub.publish(msg)



if __name__ == "__main__":
    rospy.init_node('jirwin_gate_task')
    node = GateTask(sys.argv[1:])
    rospy.spin()
