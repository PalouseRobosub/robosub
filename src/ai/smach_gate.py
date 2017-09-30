#!/usr/bin/python
import rospy
from util import *
from rs_yolo.msg import DetectionArray
from robosub.msg import control
from SubscribeState import SubscribeState
from control_wrapper import control_wrapper
import smach
import smach_ros

#Based on move_forward from forward_until.py with tweaks for vision
class move_to_gate(SubscribeState):
    def __init__(self, vision_label, poll_rate=10):
        SubscribeState.__init__(self, "vision", DetectionArray, self.callback,
            outcomes=['success'])
        self.vision_label = vision_label
        self._poll_rate = rospy.Rate(poll_rate)
        self.forward_error = rospy.get_param("ai/move_to_gate/forward_error")

    def callback(self, detectionArray):
        c = control_wrapper()
        c.levelOut()
        # forward_error is a parameter
        c.forwardError(forward_error)

        detections = filterByLabel(detectionArray.detections,
            self.vision_label)
        vision_result = getNMostProbable(detections, 2, thresh=0.5)
        normalize(vision_result)

        #move while forward until we see a gate post
        if vision_result is None:
            c.publish()
            self._poll_rate.sleep()
        return 'success'

# Lost state for when AI feels lost
class lost(SubscribeState):
    def __init__(self, vision_label):
        SubscribeState.__init__(self, "vision", DetectionArray, self.callback,
            outcomes=['2 posts', '1 post', '1 post + marker', 'none'])
            self.vision_label = vision_label

    def callback(self, detectionArray):

        detections_gate = filterByLabel(detectionArray.detections,
            self.vision_label)
        detections_path = filterByLabel(detectionArray.detections,
            'path_marker')

        vision_result_gate = getNMostProbable(detections_gate, 2, thresh=0.5)
        vision_result_path = getNMostProbable(detections_gate, 2, thresh=0.5)


        if len(vision_result_gate) == 2:
            return '2 posts'
        elif len(vision_result_gate) == 1:
            return '1 post'
        elif len(vision_result_gate) == 0:
            'none'
        elif len(vision_result_gate) == 1 && len(vision_result_path) != 0:
            '1 post + marker'

# TODO State for searching gate posts
class search(SubscribeState):
    def __init__(self, vision_label, direction):
        SubscribeState.__init__(self, "vision", DetectionArray, self.callback,
            outcomes=['2 posts', 'nothing', '1 post'])
            self.vision_label = vision_label
            self.direction = direction
            self.yaw = rospy.get_param("ai/search_gate/yaw_speed_factor")
            self._poll_rate = rospy.Rate(poll_rate)

    def callback(self, detectionArray):
        c = control_wrapper()
        c.levelOut()
        c.forwardError(0.0)

        detections = filterByLabel(detectionArray.detections,
            self.vision_label)
        vision_result = detection.getNMostProbable(detections, 2, thresh=0.5)

        if self.direction == 'right':
            self.yaw = -self.yaw

        c.yawLeftRelative(self.yaw)
        while len(vision_result) != 2 || len(vision_result) != 0:
            c.publish()
            self._poll_rate.sleep()

        if len(vision_result) == 0:
            return 'see nothing'
        elif len(vision_result) == 2:
            return '2 posts'

#TODO State for centering between gate posts or moving while being centered
class move_center(SubscribeState):
    def __init__(self, vision_label):
        SubscribeState.__init__(self, "vision", DetectionArray, self.callback,
            outcomes=['centered', 'lost'])
            self.vision_label = vision_label
            self.errorGoal = rospy.get_param("ai/center/errorGoal")
            self.center_offset = rospy.get_param("ai/center/center_offset")
            self.yaw_factor = rospy.get_param("ai/center/yaw_factor")
            self.dive_factor = rospy.get_param("ai/center/dive_factor")


    def callback(self, detectionArray):
        c = control_wrapper()
        c.levelOut()
        c.forwardError(0.0)

        detections = filterByLabel(detectionArray.detections,
            self.vision_label)
        vision_result = detections.getNMostProbable(detections, 2, thresh=0.5)

        gateXPos = (vision[0].x + vision[1].x) / 2
        gateYPos = (vision[0].y + vision[1].y) / 2

        if abs(gateXPos) > self.errorGoal:
            # If we are not centered by yaw
            yaw_left = gateXPos * self.yaw_factor
            c.yawLeftRelative(yaw_left)
        elif abs(gateYPos) > self.errorGoal:
            # If our depth is not enough for centering
            dive = gateYPos * self.dive_factor
            #TODO pushing dive using control wrapper
        elif (((vision[0].width * vision[0].height) +
            (vision[1].width * vision[1].height)) > self.distanceGoal):
            return 'centered'

        if len(vision_result) < 2:
            return 'lost'
        c.publish()

# TODO High state machine of gate task
class gate_task(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success'])

        with self:
            smach.StateMachine.add('FORWARD_UNTIL_FOUND_GATE',
                move_to_gate('start_gate'), transitions={'success': 'CENTER'})

if __name__ == '__main__':
    rospy.init_node('gate_task')
    sm = gate_task()

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcomes = sm.execute()
    sis.stop()
