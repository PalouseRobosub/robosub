#!/usr/bin/python
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool
from robosub_msgs.msg import DetectionArray
from util import *
from control_wrapper import control_wrapper
from SubscribeState import SubscribeState
from blind_movement import move_forward

class track_buoy(SubscribeState):
    def __init__(self, vision_label):
        SubscribeState.__init__(self, "vision/left", DetectionArray,
                               self.callback, outcomes=['success', 'lost_buoy'],
                               setup_callback=self.setup)
        self.vision_label = vision_label

        # How close to center the buoy needs to be (abs)
        self.errorGoal = rospy.get_param("ai/hit_buoy/track_buoy/errorGoal")

        # How close the sub has to get before reversing
        self.distGoal = rospy.get_param("ai/hit_buoy/track_buoy/distGoal")

        self.yaw_speed_factor = \
            rospy.get_param("ai/hit_buoy/track_buoy/yaw_speed_factor")
        self.dive_speed_factor = \
            rospy.get_param("ai/hit_buoy/track_buoy/dive_speed_factor")

        self.center_offset = \
            rospy.get_param("ai/hit_buoy/track_buoy/center_offset")

    def setup(self):
        self.lost_counter = 0
        self.lost_counter_max = 5

    def callback(self, msg, userdata):
        c = control_wrapper()
        c.levelOut()
        c.forwardError(0.0)

        detections = filterByLabel(msg.detections, self.vision_label)

        vision_result = getMostProbable(detections, thresh=0.5)

        normalize(vision_result)


        if vision_result is not None:
            vision_result.x -= self.center_offset
            self.lost_counter = 0
            if abs(vision_result.x) > self.errorGoal:
                    # Center X (yaw) first
                    # Calculation found by testing, will be updated with
                    # magnitude changes.
                    yaw_left = (vision_result.x *
                                    (1 - (vision_result.width *
                                          vision_result.height * 10)) *
                                    self.yaw_speed_factor)
                    c.yawLeftRelative(yaw_left)
                    rospy.logdebug("Yaw relative: {}".format(yaw_left))
                    # Maintain depth
                    c.diveRelative(0)
            else:
                # Now center Y (Dive) and maintain yaw
                c.yawLeftRelative(0)
                # Calculation found by testing, will be updated with magnitude
                # changes.
                dive = (vision_result.y *
                            ((1 - (vision_result.width * vision_result.height *
                                   10)) * self.dive_speed_factor))
                rospy.logdebug("Dive relative: {}".format(dive))
                c.diveRelative(dive)

            volume = abs(vision_result.width * vision_result.height)

            if (volume > self.distGoal*2):
                self.exit("success")
                return
            # if we are far away and roughly centered on the buoy
            elif (abs(vision_result.x) < self.errorGoal) and \
                 (abs(vision_result.y) < self.errorGoal):

                # if we are close enough, transition to ram state
                volume = abs(vision_result.width * vision_result.height)
                rospy.logdebug("volume: {}".format(volume))
                if volume > self.distGoal:
                    self.exit("success")
                    return
                else:  # move slowly forward
                    c.forwardError(0.2)
        else:  # we can't see the buoy
            self.lost_counter += 1
            if self.lost_counter > self.lost_counter_max:
                self.exit("lost_buoy")
            return

        c.publish()

class find_buoy(SubscribeState):
    def __init__(self, vision_label):
        SubscribeState.__init__(self, "vision/left", DetectionArray,
                               self.callback, outcomes=['success'])
        self.vision_label = vision_label
        self.errorGoal = rospy.get_param("ai/find_buoy/errorGoal")
        self.yaw_speed_factor = rospy.get_param("ai/find_buoy/yaw_speed_factor")

    def callback(self, msg, userdata):
        c = control_wrapper()
        c.levelOut()
        c.forwardError(0.0)
        c.diveAbsolute(-1)

        detections = filterByLabel(msg.detections, self.vision_label)

        vision_result = getMostProbable(detections, thresh=0.5)

        normalize(vision_result)

        if vision_result is None:
            # if we can't see the object in question, yaw
            yaw_left = 5
            c.yawLeftRelative(yaw_left)
        else:  # we can see the object
            # if we aren't close to center
            if abs(vision_result.x) > self.errorGoal:
                    # Center X (yaw) first
                    # Calculation found by testing, will be updated with
                    # magnitude changes.
                    yaw_left = (vision_result.x *
                                    (1 - (vision_result.width *
                                          vision_result.height * 10)) *
                                    self.yaw_speed_factor)
                    c.yawLeftRelative(yaw_left)
                    rospy.logdebug("Yaw relative: {}".format(yaw_left))
            else:  # object is close to center of screen, this state is done
                self.exit("success")

        c.publish()

class hit_buoy(smach.StateMachine):
    def __init__(self, vision_label):
        smach.StateMachine.__init__(self, outcomes=['success'])
        with self:
            smach.StateMachine.add("FIND_BUOY", find_buoy(vision_label),
                                  transitions={'success': 'TRACKING'})

            smach.StateMachine.add("TRACKING", track_buoy(vision_label),
                                  transitions={'success': 'RAM_BUOY',
                                              'lost_buoy': 'FIND_BUOY'})

            ram_time = rospy.get_param("ai/hit_buoy/ram_time_seconds")
            ram_speed = rospy.get_param("ai/hit_buoy/ram_speed")
            smach.StateMachine.add("RAM_BUOY",
                                  move_forward(time=ram_time, value=ram_speed),
                                  transitions={'success': 'success'})
