#!/usr/bin/env python

import rospy
from robosub.msg import visionPosArray as vision_pos_array
from robosub.msg import control

class BuoyTask():

    def __init__(self):
        rospy.loginfo("Init done")
        self.pub = rospy.Publisher('control', control, queue_size=1)
        self.sub = rospy.Subscriber('vision/buoys/red', vision_pos_array,
                                    self.callback)
        self.state = "SEARCHING"
        self.hitTime = None
        # How long the sub moves backward for
        self.duration = 2
        # How close to center the buoy needs to be (abs)
        self.errorGoal = 0.1
        # How close the sub has to get before reversing
        self.distGoal = 0.01

    def callback(self, vision_result):
        msg = control()

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
                msg.forward = -10

            # Both instances should maintain yaw and dive
            msg.yaw_state = control.STATE_RELATIVE
            msg.yaw_left = 0
            msg.dive_state = control.STATE_RELATIVE
            msg.dive = 0

        # Check for empty message, we can't see a buoy
        elif len(vision_result.data) < 1:
            if self.state is "TRACKING":
                # We were tracking the buoy, but we've lost it now...
                # TODO: Decide what to do when this happens
                rospy.logerr("Lost buoy...")
            self.state = "SEARCHING"
            # Spin 10 degrees
            msg.yaw_state = control.STATE_RELATIVE
            msg.yaw_left = 10
            # Don't move forward and maintain depth
            msg.forward_state = control.STATE_ERROR
            msg.forward = 0
            msg.dive_state = control.STATE_RELATIVE
            msg.dive = 0
        # We have found the buoy and it is centered
        elif (abs(vision_result.data[0].xPos) < self.errorGoal and
              abs(vision_result.data[0].yPos) < self.errorGoal):
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
            if vision_result.data[0].magnitude < self.distGoal:
                msg.forward = 10
                self.state = "RAMMING"
                rospy.loginfo("{} from goal".format(self.distGoal -
                              vision_result.data[0].magnitude))
            else:
                # Since the buoy has been rammed, begin end procedure
                msg.forward = 0
                self.state = "HIT"
                self.hitTime = rospy.get_rostime()
                rospy.loginfo("Hit time: {}".format(self.hitTime))

        else:
            # We see a buoy, but it is not centered.
            self.state = "TRACKING"
            if abs(vision_result.data[0].xPos) > self.errorGoal:
                # Center X (yaw) first
                msg.yaw_state = control.STATE_RELATIVE
                # Calculation found by testing, will be updated with magnitude
                # changes.
                msg.yaw_left = (vision_result.data[0].xPos *
                                (1 - (vision_result.data[0].magnitude * 10)) *
                                (-50))
                rospy.loginfo("Yaw error: {}".format(msg.yaw_left))
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
                msg.dive = (vision_result.data[0].yPos *
                            ((1 - (vision_result.data[0].magnitude * 10)) * -5))
                rospy.loginfo("Dive error: {}".format(msg.dive))

        self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node('jirwin_buoy_follower')
    node = BuoyTask()
    rospy.spin()
