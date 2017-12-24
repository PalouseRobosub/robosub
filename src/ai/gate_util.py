#!/usr/bin/python
import rospy
from util import *
from rs_yolo.msg import DetectionArray
from robosub.msg import control
from robosub.msg import Euler
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
                               outcomes=['search', '2 posts'],
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
        if(len(vision_result)):
            self.exit('2 posts')
        else:
            self.exit('search')

# This state is designed to record orientation from topic /pretty/orientation
class take_orientation(SubscribeState):
    def __init__(self):
        SubscribeState.__init__(self, "/pretty/orientation", Euler,
                               self.callback, outcomes=['success'],
                               input_keys=['direction'],
                               output_keys=['direction'])
    def callback(self, direction, userdata):
        userdata.direction = direction
        self.exit('success')

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

        with self:
            smach.StateMachine.add("LOST", lost(label),
                                  transitions={'2 posts': 'TAKE_ORIENTATION',
                                              'search': 'SEARCH'},
                                  remapping={'direction': 'direction'})

            smach.StateMachine.add("SEARCH", yaw_from_current_to_given(),
                                  transitions={'success': 'LOST'},
                                  remapping={'direction': 'direction'})

            smach.StateMachine.add("TAKE_ORIENTATION", take_orientation(),
                                  transitions={'success': 'success'},
                                  remapping={'direction': 'direction'})

# State for centering between gate posts or moving while being centered
class center(SubscribeState):
    def __init__(self, vision_label):
        SubscribeState.__init__(self, "vision", DetectionArray, self.callback,
                               outcomes=['centered', 'lost'],
                               input_keys=['direction'],
                               output_keys=['direction'])
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

        # Saving detection Array for next state, in case we loose post.
        userdata.detectionsArray = vision_result;

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
                               outcomes=['ready', 'not centered', 'lost'],
                               input_keys=['direction'],
                               output_keys=['direction'])
        self.vision_label = vision_label
        self.distanceGoal = rospy.get_param("ai/move_forward/distanceGoal")
        self.error_goal = rospy.get_param("ai/move_forward/error_goal")
        self.forward_speed = rospy.get_param("ai/move_forward/forward_speed")

    def callback(self, detectionArray, userdata):
        c = control_wrapper()
        vision_result = getNMostProbable(detections, 2, thresh=0.5)

        if len(vision_result) < 2:
            self.exit('lost')
            return 'lost'

        # Saving detection Array for next state, in case we loose post.
        userdata.detectionsArray = vision_result;

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
        c.publish()

# State that turns to the yaw that is being passed.
# Requires direction to contain something to turn.
class turn_to_yaw(smach.State):
    def __init__(self, value, poll_rate=10):
        smach.State.__init__(self, outcomes=["success", "continue"],
                            input_keys=['direction'],
                            output_keys=['direction'])
        self.value = value
        self._poll_rate = rospy.Rate(poll_rate)
        self.bearing_error = rospy.get_param("ai/bearing_error")
        # wait for time to be non-zero
        while(rospy.Time.now() == rospy.Time(0)):
            rospy.sleep(1.0/poll_rate)

    def execute(self, userdata):
        c = control_wrapper()
        c.levelOut()
        print(type(userdata.direction.yaw))
        c.yawLeftRelative(self.value - userdata.direction.yaw)
        if (abs(self.value - userdata.direction.yaw) < self.bearing_error):
            c.publish()
            self._poll_rate.sleep()
            return 'continue'
        return 'success'

# State Machine that can turn to desired bearing that is being passed as an arg
class yaw_from_current_to_given(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success'])
        self.value = userdata.direction.yaw
        with self:
            smach.StateMachine.add("ORIENTATION", take_orientation(),
                                  transitions={'success': 'TURN_TO_BEARING'},
                                  remapping={'direction': 'direction'})
            smach.StateMachine.add("TURN_TO_BEARING", turn_to_yaw(self.value),
                                  transitions={'continue': 'ORIENTATION',
                                  'success': 'success'},
                                  remapping={'direction': 'direction'})
