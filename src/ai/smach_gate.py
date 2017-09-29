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

    def callback(self, detectionArray):
        c = control_wrapper()
        c.levelOut()
        #10 is an arbirtary number
        c.forwardError(10)

        detections = filterByLabel(detectionArray.detections, self.vision_label)
        vision_result = getMostProbable(detections, thresh=0.5)
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

    def callback(self, detectionArray):

        detections_gate = filterByLabel(detectionArray.detections,
            self.vision_label)
        detections_path = filterByLabel(detectionArray.detections, 'path_marker')

        if len(detections_gate) == 2:
            return '2 posts'
        elif len(detections_gate) == 1:
            return '1 post'
        elif len(detections_gate) == 0:
            'none'
        elif len(detections_gate) == 1 && len(detections_path) != 0:
            '1 post + marker'

# TODO State for searching gate posts
class search(SubscribeState):
    def __init__(self, direction):
        SubscribeState.__init__(self, "vision", DetectionArray, self.callback,
            outcomes=['marker'])

    def callback(self, detectionArray)

# TODO High state machine of gate task
class gate_task(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success'])

        with self:
            smach.StateMachine.add('FORWARD UNTIL FOUND GATE',
                move_to_gate('start_gate'), transitions={'success': 'success'})

if __name__ == '__main__':
    rospy.init_node('gate_task')
    sm = gate_task()

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcomes = sm.execute()
    sis.stop()
