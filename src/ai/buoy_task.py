#!/usr/bin/env python

import rospy
from rs_yolo.msg import DetectionArray as detection_array
from robosub.msg import control
from util import *

import sys

class BuoyTask():

    def __init__(self, label_name):
        rospy.loginfo("Init done")
        self.pub = rospy.Publisher('control', control, queue_size=1)
        self.sub = rospy.Subscriber('vision', detection_array,
                                    self.callback)
        self.state = "SEARCHING"
        self.hitTime = None
        # How long the sub moves backward for
        self.duration = 5
        # How close to center the buoy needs to be (abs)
        self.errorGoal = 0.1
        # How close the sub has to get before reversing
        self.distGoal = 0.06

        # How close the sub has to get before blindly ramming
        self.distGoal = 0.01
        # How long the sub should blindly ram for before reversing
        self.blindRamDuration = 2

        self.blindRamTime = None

        
        ######## Speed settings #######
        # The speed to send to ram the buoys
        self.ram_speed = 0.5
        # The factor to multiply the error in the image by to get a resulting
        # speed (yaw and dive)
        self.yaw_speed_factor = -25
        self.dive_speed_factor = -1 

        self.yaw_search_speed = 5

        # The absolute value speed to use when backing
        self.reverse_speed = 0.5

        self.label_name = label_name

    def callback(self, detection):
        msg = control()

        detections = filterByLabel(detection.detections, self.label_name)

        vision_result = getMostProbable(detections, thresh=0.5)

        normalize(vision_result)

        # Maintain roll and pitch of 0
        msg.roll_state = control.STATE_ABSOLUTE
        msg.roll_right = 0
        msg.pitch_state = control.STATE_ABSOLUTE
        msg.pitch_down = 0
        # Don't move forward
        msg.forward_state = control.STATE_ERROR
        msg.forward = 0

        rospy.loginfo("state: {}".format(self.state))

        # When we've hit the buoy, prepare to reset for the next task
        if self.state is "HIT" and self.hitTime is not None:
            msg.forward_state = control.STATE_ERROR
            # TODO: Improve logic for after hitting buoy to be more useful
            # If we have moved for long enough, end task
            if self.hitTime + rospy.Duration(self.duration) < \
               rospy.get_rostime():
                msg.forward = 0
                rospy.loginfo("Hit buoy")
                self.pub.publish(msg)
                rospy.signal_shutdown(0)
            # Otherwise continue to reverse
            else:
                rospy.loginfo("Reversing from buoy")
                msg.forward = -self.reverse_speed

            # Both instances should maintain yaw and dive
            msg.yaw_state = control.STATE_RELATIVE
            msg.yaw_left = 0
            msg.dive_state = control.STATE_RELATIVE
            msg.dive = 0

        # Check for empty message, we can't see a buoy
        elif vision_result is None:
            if self.state is "TRACKING":
                # We were tracking the buoy, but we've lost it now...
                # TODO: Decide what to do when this happens
                rospy.logerr("Lost buoy...")
            self.state = "SEARCHING"
            # Spin 10 degrees
            msg.yaw_state = control.STATE_RELATIVE
            msg.yaw_left = self.yaw_search_speed
            # Don't move forward and maintain depth
            msg.forward_state = control.STATE_ERROR
            msg.forward = 0
            msg.dive_state = control.STATE_RELATIVE
            msg.dive = 0
        # We have found the buoy and it is centered
        elif (abs(vision_result.x) < self.errorGoal and
              abs(vision_result.y) < self.errorGoal):
            # Maintain yaw and dive
            msg.forward_state = control.STATE_ERROR
            msg.yaw_state = control.STATE_RELATIVE
            msg.yaw_left = 0
            msg.dive_state = control.STATE_RELATIVE
            msg.dive = 0
            rospy.loginfo("Centered on buoy, now ramming...")
            # When we aren't close enough to the buoy, begin ramming by moving
            # straight forward
            # Magnitude is currently a crude distance measurement relating the
            # area of the object to the area of the image. This will need to be
            # updated when the magnitude calculation is replaced by stereo
            # disparity calculations.
            if vision_result.width * vision_result.height < self.distGoal:
                msg.forward = self.ram_speed
                self.state = "RAMMING"
                rospy.loginfo("{} from goal".format(self.distGoal -
                              vision_result.width * vision_result.height))
            else:
                if self.blindRamTime is None:
                    self.blindRamTime = rospy.get_rostime()
                elif self.blindRamTime + rospy.Duration(self.blindRamDuration)\
                     < rospy.get_rostime():
                    # We need to blindly ram
                    msg.forward = self.ram_speed
                    self.state = "RAMMING"
                    rospy.loginfo("Ramming blindly...")

                else:
                    # Since the buoy has been rammed, begin end procedure
                    msg.forward = 0
                    self.state = "HIT"
                    self.hitTime = rospy.get_rostime()
                    rospy.loginfo("Hit time: {}".format(self.hitTime))

        else:
            # We see a buoy, but it is not centered.
            self.state = "TRACKING"
            if abs(vision_result.x) > self.errorGoal:
                # Center X (yaw) first
                msg.yaw_state = control.STATE_RELATIVE
                # Calculation found by testing, will be updated with magnitude
                # changes.
                msg.yaw_left = (vision_result.x *
                                (1 - (vision_result.width *
                                      vision_result.height * 10)) *
                                self.yaw_speed_factor)
                rospy.loginfo("Yaw relative: {}".format(msg.yaw_left))
                # Maintain depth
                msg.dive_state = control.STATE_RELATIVE
                msg.dive = 0
            else:
                # Now center Y (Dive) and maintain yaw
                msg.yaw_state = control.STATE_RELATIVE
                msg.yaw_left = 0
                msg.dive_state = control.STATE_RELATIVE
                # Calculation found by testing, will be updated with magnitude
                # changes.
                msg.dive = (vision_result.y *
                            ((1 - (vision_result.width * vision_result.height *
                                   10)) * self.dive_speed_factor))
                rospy.loginfo("Dive relative: {}".format(msg.dive))
>>>>>>> Stashed changes

        self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node('buoy_task')
    node = BuoyTask(sys.argv[1])
    rospy.spin()
