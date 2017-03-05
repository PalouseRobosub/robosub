#!/usr/bin/env python

import sys
import getopt
import rospy
from robosub.msg import visionPosArray as vision_pos_array
from robosub.msg import control

class GateTask():
    def __init__(self, commandArgs):
        rospy.loginfo("Init done")

        # Set blind forward duration default to 1 second
        self.duration = 1

        # Use command line arguments to get duration
        try:
            opts, args = getopt.getopt(commandArgs, "hd:", ["duration="])
        except getopt.GetoptError:
            print "Usage: gate_jirwin.py -d <duration>"
            sys.exit(2)
        for opt, arg in opts:
            if opt == '-h':
                # Print usage message
                print "Usage: gate_jirwin.py -d <duration>"
                print "       or"
                print "       gate_jirwin.py --duration=<duration>"
                sys.exit()
            elif opt in ("-d", "--duration"):
                # Fetch duration
                self.duration = int(arg)
                rospy.loginfo("Setting duration to {} seconds.".format(
                              self.duration))

        self.pub = rospy.Publisher('control', control, queue_size=1)
        self.sub = rospy.Subscriber('vision/start_gate', vision_pos_array,
                                    self.callback)
        # Set initial state to LOST as we have no idea where we are
        self.state = "LOST"
        # How far off center the gate should be
        self.errorGoal = 0.1
        # Used to determine if moving we were moving left or right last
        self.prev_yaw = 0
        self.throughTime = None

    # Called when there are no gate posts found by the vision
    def noneFound(self, msg, vision):
        # Maintain dive at -2 meters (Perhaps this could be made a parameter)
        msg.yaw_state = control.STATE_ERROR
        msg.forward_state = control.STATE_ERROR
        msg.forward = 0
        msg.dive_state = control.STATE_ABSOLUTE
        msg.dive = -2

        if self.state is "LOST":
            # Rotate left by default if no data
            msg.yaw_left = 10
        elif self.state is "SEARCHING LEFT":
            # We had one post and lost it, go back right to find it again
            self.state = "SEARCHING RIGHT"
            msg.yaw_left = -10
        elif self.state is "SEARCHING RIGHT":
            # We had one post and lost it, go back left to find it again
            self.state = "SEARCHING LEFT"
            msg.yaw_left = 10
        elif self.state is "TRACKING":
            # We were seeing both posts and lost them both at once!
            # What happened!?
            # TODO: Determine if there's something better to do here
            self.state = "LOST"
            msg.yaw_left = 0
        else:
            # We were in the THROUGH state so we have passed through the gate
            pass

    # Called when there is only one gate post found
    def oneFound(self, msg, vision):
        if self.state is "LOST":
            if vision.data[0].xPos < 0:
                # If the post is on the left of the image, search right for the
                # other one.
                self.state = "SEARCHING_RIGHT"
                msg.yaw_left = -10
            else:
                # If the post is on the right of the image, search left for the
                # other one.
                self.state = "SEARCHING_LEFT"
                msg.yaw_left = 10
        elif self.state is "SEARCHING_LEFT" or self.state is "SEARCHING_RIGHT":
            # If we were searching one way or another, there is no new info
            # to make decisions on. Just keep going.
            pass
        elif self.state is "TRACKING":
            # We had two posts and lost one. Go find it.
            msg.forward_state = control.STATE_ERROR
            msg.forward = 0
            msg.yaw_state = control.STATE_ERROR
            # Reverse directions. As our state itself doesn't contain that info,
            # we need to keep track of the previous yaw value.
            # Note: This could be replaced by further complicating the state
            #       values
            msg.yaw_left = self.prev_yaw * -1
            if msg.yaw_left > 0:
                # We are now yawing left
                self.state = "SEARCHING_LEFT"
            elif msg.yaw_left < 0:
                # We are now yawing right
                self.state = "SEARCHING_RIGHT"
            else:
                # Our relative yaw was zero, use the vision to make decisions
                if vision.data[0].xPos < 0:
                    # The post is on the left of the image, so yaw right to
                    # find the other post
                    self.state = "SEARCHING_RIGHT"
                    msg.yaw_left = 10
                else:
                    # The post is on the right of the image, so yaw left to
                    # find the other post
                    self.state = "SEARCHING_LEFT"
                    msg.yaw_left = -10

    # Called when we see both posts
    def twoFound(self, msg, vision):
        # No matter what we were doing, we are now TRACKING the gate
        self.state = "TRACKING"

        # Calculate the center of the gate itself by taking the median of the
        # individual post X and Y positions
        gateXPos = (vision.data[0].xPos + vision.data[1].xPos) / 2
        gateYPos = (vision.data[0].yPos + vision.data[1].yPos) / 2

        if abs(gateXPos) > self.errorGoal:
            # Center X (yaw) first
            msg.yaw_state = control.STATE_RELATIVE
            # The scalar of 50 was determined through testing and will need to
            # change when stereo vision is implemented
            msg.yaw_left = gateXPos * -50
            rospy.loginfo("Yaw output: {}".format(msg.yaw_left))
        elif abs(gateYPos) > self.errorGoal:
            # Center Y (dive) second
            msg.dive_state = control.STATE_RELATIVE
            # The scalar of 50 was determined through testing and will need to
            # change when stereo vision is implemented
            msg.dive = gateYPos * -50
            rospy.loginfo("Dive output: {}".format(msg.dive))
        else:
            # We are centered on the gate
            if (vision.data[0].magnitude + vision.data[1].magnitude) > 0.012:
                # We are within a good distance from the gate.
                # The value of 0.012 was determined through testing and needs
                # to be updated when stereo vision is implemented

                # Set our state to THROUGH. We will now move forward through
                # the gate
                self.state = "THROUGH"
            else:
                # We aren't close enough to the gate to guarantee being centered
                msg.yaw_state = control.STATE_RELATIVE
                msg.yaw_left = 0
                msg.dive_state = control.STATE_RELATIVE
                msg.dive = 0
                # Move forward
                msg.forward_state = control.STATE_ERROR
                msg.forward = 10

    # Called when a vision input is received.
    # Perhaps adding some safeguard timeouts so if a message isn't recieved
    # within a specific amount of time, stop motion.
    def callback(self, vision_result):
        # A dictionary of function pointers mapping the number of gate posts
        # found to the proper function to call
        performTask = {0: self.noneFound,
                       1: self.oneFound,
                       2: self.twoFound,
                       }

        # Construct the control message
        msg = control()

        rospy.loginfo("state: {}".format(self.state))

        if self.state is not "THROUGH":
            # If we haven't gotten to our distance goal from the gate, perform
            # our other logic by finding the number of gate posts present in
            # sight and calling the appropriate function using the dictionary
            # of function pointers.
            numFound = len(vision_result.data)
            performTask[numFound](msg, vision_result)
        elif not self.throughTime:
            # We have moved within our distance goal, but have not completed
            # our blind movement

            # Initialize the completion time to the current time
            self.throughTime = rospy.get_rostime()
            rospy.loginfo("Through time: {}".format(self.throughTime))

        # We will never be changing roll or pitch, so set them to absolute 0
        msg.roll_state = control.STATE_ABSOLUTE
        msg.roll_right = 0
        msg.pitch_state = control.STATE_ABSOLUTE
        msg.pitch_down = 0

        # always maintain depth at 1 meter by default (Perhaps this could be
        # made into a parameter)
        msg.dive_state = control.STATE_ABSOLUTE
        msg.dive = -1

        if self.state is "THROUGH" and self.throughTime is not None:
            # We have completed our distance goal and have a completion time,
            # this means that we may need to keep moving forward
            msg.forward_state = control.STATE_ERROR
            if self.throughTime + rospy.Duration(self.duration) <\
               rospy.get_rostime():
                # We have surpassed the blind movement duration and can kill
                # this process
                msg.forward = 0
                rospy.loginfo("Through gate")
                self.pub.publish(msg)
                rospy.signal_shutdown(0)
            else:
                # We have not reached the desired blind movement time
                # and should be moving forward
                rospy.loginfo("Moving through gate")
                msg.forward = 10

            # Maintain our centered yaw and dive
            msg.yaw_state = control.STATE_RELATIVE
            msg.yaw_left = 0
            msg.dive_state = control.STATE_RELATIVE
            msg.dive = 0

        # Update our previous yaw value to what our current yaw value is
        self.prev_yaw = msg.yaw_left
        self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node('jirwin_gate_task')
    node = GateTask(sys.argv[1:])
    rospy.spin()
