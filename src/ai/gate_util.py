#!/usr/bin/python
import rospy
from util import *
from rs_yolo.msg import DetectionArray
from robosub.msg import control
from SubscribeState import SubscribeState
from control_wrapper import control_wrapper
import smach
import smach_ros

# Based on move_forward from forward_until.py with tweaks for vision
class move_to_gate(SubscribeState):
    def __init__(self, vision_label):
        SubscribeState.__init__(self, "vision", DetectionArray, self.callback,
                               outcomes=['success'])
        self.vision_label = vision_label
        self.forward_speed = rospy.get_param("ai/move_to_gate/forward_speed")

    def callback(self, detectionArray, userdata):
        c = control_wrapper()
        c.levelOut()
        c.forwardError(self.forward_speed)

        detections = filterByLabel(detectionArray.detections,
                                  self.vision_label)
        vision_result = getNMostProbable(detections, 2, thresh=0.5)
        normalize(vision_result)
        # move forward until we see a gate task post
        if len(vision_result) == 0:
            c.publish()
        else:
            self.exit("success")

# Lost state for when AI feels lost
# When we do not see any of gate posts at all
class lost(SubscribeState):
    def __init__(self, vision_label, poll_rate=10):
        SubscribeState.__init__(self, "vision", DetectionArray, self.callback,
                               outcomes=['1 post', 'none', '2 posts'],
                               input_keys=['direction'],
                               output_keys=['direction'])
        self.vision_label = vision_label
        self.yaw = rospy.get_param("ai/search_gate/yaw_speed_factor")
        self._poll_rate = rospy.Rate(poll_rate)

    def callback(self, detectionArray, userdata):
        c = control_wrapper()
        c.forwardError(0.0)
        detections = filterByLabel(detectionArray.detections,
                                  self.vision_label)

        vision_result = getNMostProbable(detections, 2, thresh=0.5)

        currentYaw = 0
        if userdata.direction == 'right':
            currentYaw = -self.yaw
        else:
            currentYaw = self.yaw

        c.yawLeftRelative(currentYaw)
        rospy.logdebug("objects detected: {}".format(len(vision_result)))

        if len(vision_result) == 0:
            rospy.logdebug("search direction {}".format(userdata.direction))
            rospy.logdebug("yaw: {}".format(currentYaw))
            c.publish()
            self._poll_rate.sleep()
            self.exit('none')
        elif len(vision_result) == 1:
            self.exit('1 post')
        else:
            self.exit('2 posts')

# Flips direction of search
class flip(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'],
                            input_keys=['direction'],
                            output_keys=['direction'])

    def execute(self, userdata):
        if userdata.direction == 'right':
            userdata.direction = 'left'
        else:
            userdata.direction = 'right'
        return 'success'

# State for searching gate posts. Comes after state Lost, so it searches,
# and yaws same direction as it was in Lost state
# By default yaws to the right until
# it either loses all of posts or finds second.
class search(SubscribeState):
    def __init__(self, vision_label, poll_rate=10):
        SubscribeState.__init__(self, "vision", DetectionArray, self.callback,
                               outcomes=['2 posts', 'none', '1 post'],
                               input_keys=['direction'],
                               output_keys=['direction'])
        self.vision_label = vision_label
        self.yaw = rospy.get_param("ai/search_gate/yaw_speed_factor")
        self._poll_rate = rospy.Rate(poll_rate)

    def callback(self, detectionArray, userdata):
        c = control_wrapper()
        c.forwardError(0.0)

        detections = filterByLabel(detectionArray.detections,
                                  self.vision_label)
        vision_result = getNMostProbable(detections, 2, thresh=0.5)

        rospy.logdebug("objects detected: {}".format(len(vision_result)))

        currentYaw = 0
        if userdata.direction == 'right':
            currentYaw = -self.yaw
        else:
            currentYaw = self.yaw

        c.yawLeftRelative(currentYaw)
        if len(vision_result) == 1:
            c.publish()
            self._poll_rate.sleep()
            self.exit('1 post')
        elif len(vision_result) == 2:
            self.exit('2 posts')
        else:
            rospy.logdebug("search direction {}".format(userdata.direction))
            self.exit('none')


# State machine for searching gate posts
class Search_for_gates(smach.StateMachine):
    def __init__(self, label):
        smach.StateMachine.__init__(self, outcomes=['success'])
        self.userdata.direction = 'right'

        with self:
            smach.StateMachine.add("LOST", lost(label),
                                  transitions={'1 post': 'SEARCH',
                                              'none': 'LOST',
                                              '2 posts': 'SEARCH'},
                                  remapping={'direction': 'direction'})

            smach.StateMachine.add("FLIP", flip(),
                                  transitions={'success': 'LOST'},
                                  remapping={'direction': 'direction'})

            smach.StateMachine.add("SEARCH", search(label),
                                  transitions={'2 posts': 'success',
                                              'none': 'FLIP',
                                              '1 post': 'SEARCH'},
                                  remapping={'direction': 'direction'})

# State for centering between gate posts or moving while being centered
class center(SubscribeState):
    def __init__(self, vision_label):
        SubscribeState.__init__(self, "vision", DetectionArray, self.callback,
                               outcomes=['centered', 'lost'])
        self.vision_label = vision_label
        self.error_goal = rospy.get_param("ai/center/error_goal")
        self.yaw_factor = rospy.get_param("ai/center/yaw_factor")
        self.dive_factor = rospy.get_param("ai/center/dive_factor")


    def callback(self, detectionArray, userdata):
        c = control_wrapper()
        c.levelOut()

        detections = filterByLabel(detectionArray.detections,
                                  self.vision_label)
        vision_result = getNMostProbable(detections, 2, thresh=0.5)

        if len(vision_result) < 2:
            self.exit('lost')
            return 'lost'

        gateXPos = (vision_result[0].x + vision_result[1].x) / 2
        gateYPos = (vision_result[0].y + vision_result[1].y) / 2

        rospy.logdebug(("Gate X: {}\tGate Y: {}".format(gateXPos, gateYPos)))

        if abs(gateXPos-0.5) > self.error_goal:
            # If we are not centered by yaw
            yaw_left = (gateXPos-0.5) * self.yaw_factor
            c.yawLeftRelative(yaw_left * 60)
            rospy.logdebug("trying to yaw: {}".format(yaw_left * 60))
        elif abs(gateYPos-0.5) > self.error_goal:
            # If our depth is not enough for centering
            dive = (gateYPos-0.5) * self.dive_factor
            rospy.logdebug("trying to dive: {}".format(dive))
            c.diveRelative(dive)
        else:
            self.exit('centered')

        c.publish()

# State for moving forward while checking if we are centered
# In future need to make this function to watch for bottom facing camera,
# slow  down or stop if it gets close to path marker.
class move_forward_centered(SubscribeState):
    def __init__(self, vision_label):
        SubscribeState.__init__(self, "vision", DetectionArray, self.callback,
                               outcomes=['ready', 'not centered', 'lost'])
        self.vision_label = vision_label
        self.distanceGoal = rospy.get_param("ai/move_forward/distanceGoal")
        self.error_goal = rospy.get_param("ai/move_forward/error_goal")
        self.forward_speed = rospy.get_param("ai/move_forward/forward_speed")

    def callback(self, detectionArray, userdata):
        c = control_wrapper()
        c.levelOut()
        c.forwardError(self.forward_speed)

        detections = filterByLabel(detectionArray.detections,
                                  self.vision_label)
        vision_result = getNMostProbable(detections, 2, thresh=0.5)

        if len(vision_result) < 2:
            self.exit('lost')
            return 'lost'

        gateXPos = (vision_result[0].x + vision_result[1].x) / 2
        gateYPos = (vision_result[0].y + vision_result[1].y) / 2

        if (((vision_result[0].width * vision_result[0].height) +
           (vision_result[1].width *
           vision_result[1].height)) > self.distanceGoal):
            self.exit('ready')
            return 'ready'
        if (abs(gateXPos-0.5) > self.error_goal or
           abs(gateYPos-0.5) > self.error_goal):
            self.exit('not centered')
        if len(vision_result) < 2:
            self.exit('lost')
        rospy.logdebug("trying to move forward: {}".format(self.forward_speed))
        c.publish()

# Class for strafing, based on blind_movement
class strafe(smach.State):
    def __init__(self, time, value, poll_rate=10):
        smach.State.__init__(self, outcomes=["success"])
        self.time = time
        self.value = value
        self._poll_rate = rospy.Rate(poll_rate)

        # wait for time to be non-zero
        while(rospy.Time.now() == rospy.Time(0)):
            rospy.sleep(1.0/poll_rate)

    def execute(self, userdata):
        c = control_wrapper()
        c.levelOut()
        c.strafeLeftError(self.value)
        exit_time = rospy.Time.now() + rospy.Duration(self.time)
        while not rospy.is_shutdown() and rospy.Time.now() < exit_time:
            c.publish()
            self._poll_rate.sleep()
        c.strafeLeftError(0)
        c.publish()
        self._poll_rate.sleep()
        return 'success'

# Class for turning around, based on blind_movement
class turn(smach.State):
    def __init__(self, time, value, poll_rate=10):
        smach.State.__init__(self, outcomes=["success"])
        self.time = time
        self.value = value
        self._poll_rate = rospy.Rate(poll_rate)

        # wait for time to be non-zero
        while(rospy.Time.now() == rospy.Time(0)):
            rospy.sleep(1.0/poll_rate)

    def execute(self, userdata):
        c = control_wrapper()
        c.levelOut()
        c.yawLeftRelative(self.value)
        exit_time = rospy.Time.now() + rospy.Duration(self.time)
        while not rospy.is_shutdown() and rospy.Time.now() < exit_time:
            c.publish()
            self._poll_rate.sleep()
        c.yawLeftRelative(0)
        c.publish()
        self._poll_rate.sleep()
        return 'success'

# Class for  making U turn around the post post, based on blind_movement
class u_turn(smach.State):
    def __init__(self, poll_rate=10):
        smach.State.__init__(self, outcomes=["success"])
        self.time = rospy.get_param("ai/experiment/time")
        self.yawRelative = rospy.get_param("ai/experiment/yaw_relative")
        self.forward_speed = rospy.get_param("ai/experiment/forward_speed")
        self._poll_rate = rospy.Rate(poll_rate)

        # wait for time to be non-zero
        while(rospy.Time.now() == rospy.Time(0)):
            rospy.sleep(1.0/poll_rate)

    def execute(self, userdata):
        c = control_wrapper()
        c.levelOut()
        c.yawLeftRelative(self.yawRelative)
        c.forwardError(self.forward_speed)
        exit_time = rospy.Time.now() + rospy.Duration(self.time)
        while not rospy.is_shutdown() and rospy.Time.now() < exit_time:
            c.publish()
            self._poll_rate.sleep()
        c.yawLeftRelative(0)
        c.forwardError(0)
        c.publish()
        self._poll_rate.sleep()
        return 'success'

class center_single(SubscribeState):
    def __init__(self, vision_label):
        SubscribeState.__init__(self, "vision", DetectionArray, self.callback,
                               outcomes=['centered', 'lost', 'ready'])
        self.vision_label = vision_label
        self.error_goal = rospy.get_param("ai/center/error_goal")
        self.yaw_factor = rospy.get_param("ai/center/yaw_factor")
        self.dive_factor = rospy.get_param("ai/center/dive_factor")
        self.distanceGoal =
        rospy.get_param("ai/move_forward_single/distanceGoal")

    def callback(self, detectionArray, userdata):
        c = control_wrapper()
        c.levelOut()

        detections = filterByLabel(detectionArray.detections,
                                  self.vision_label)
        vision_result = getNMostProbable(detections, 1, thresh=0.5)

        if len(vision_result) < 1:
            self.exit('lost')
            return 'lost'

        gateXPos = vision_result[0].x
        gateYPos = vision_result[0].y

        rospy.logdebug(("Gate X: {}\tGate Y: {}".format(gateXPos, gateYPos)))

        rospy.logdebug("distanceGoal is: {}".format(
                      vision_result[0].width * vision_result[0].height))
        if ((vision_result[0].width * vision_result[0].height) >
           self.distanceGoal):
            self.exit('ready')
            return 'ready'
        if abs(gateXPos-0.5) > self.error_goal:
            # If we are not centered by yaw
            yaw_left = (gateXPos-0.5) * self.yaw_factor
            c.yawLeftRelative(yaw_left * 60)
            rospy.logdebug("trying to yaw: {}".format(yaw_left * 60))
        elif abs(gateYPos-0.5) > self.error_goal:
            # If our depth is not enough for centering
            dive = (gateYPos-0.5) * self.dive_factor
            rospy.logdebug("trying to dive: {}".format(dive))
            c.diveRelative(dive)
        else:
            self.exit('centered')

        c.publish()


class move_forward_centered_single(SubscribeState):
    def __init__(self, vision_label):
        SubscribeState.__init__(self, "vision", DetectionArray, self.callback,
                               outcomes=['ready', 'not centered', 'lost'])
        self.vision_label = vision_label
        self.distanceGoal =
        rospy.get_param("ai/move_forward_single/distanceGoal")
        self.error_goal = rospy.get_param("ai/move_forward/error_goal")
        self.forward_speed = rospy.get_param("ai/move_forward/forward_speed")

    def callback(self, detectionArray, userdata):
        c = control_wrapper()
        c.levelOut()
        c.forwardError(self.forward_speed)

        detections = filterByLabel(detectionArray.detections,
                                  self.vision_label)
        vision_result = getNMostProbable(detections, 1, thresh=0.5)

        if len(vision_result) < 1:
            self.exit('lost')
            return 'lost'

        gateXPos = vision_result[0].x
        gateYPos = vision_result[0].y

        rospy.logdebug("distanceGoal is: {}".format(
                      vision_result[0].width * vision_result[0].height))
        if ((vision_result[0].width * vision_result[0].height) >
           self.distanceGoal):
            self.exit('ready')
            return 'ready'
        if (abs(gateXPos-0.5) > self.error_goal or
           abs(gateYPos-0.5) > self.error_goal):
            self.exit('not centered')
        if len(vision_result) < 1:
            self.exit('lost')
        rospy.logdebug("trying to move forward: {}".format(self.forward_speed))
        c.publish()
