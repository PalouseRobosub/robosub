#!/usr/bin/python
import rospy
from util import *
from rs_yolo.msg import DetectionArray
from robosub.msg import control
from SubscribeState import SubscribeState
from control_wrapper import control_wrapper
from blind_movement import move_forward
import smach
import smach_ros

search_direction = 'right'
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
        c.forwardError(self.forward_error)

        detections = filterByLabel(detectionArray.detections,
            self.vision_label)
        vision_result = getNMostProbable(detections, 2, thresh=0.5)
        normalize(vision_result)
        #move while forward until we see a gate post
        if vision_result == 0:
            print("Publishing")
            c.publish()
            self._poll_rate.sleep()
        self.exit("success")
        return 'success'

# Lost state for when AI feels lost
class lost(SubscribeState):
    def __init__(self, vision_label, poll_rate=10):
        SubscribeState.__init__(self, "vision", DetectionArray, self.callback,
            outcomes=['1 post', 'none', '2 posts'])
        self.vision_label = vision_label
        self.yaw = rospy.get_param("ai/search_gate/yaw_speed_factor")
        self._poll_rate = rospy.Rate(poll_rate)

    def callback(self, detectionArray):
        global search_direction
        c = control_wrapper()
        c.forwardError(0.0)
        detections = filterByLabel(detectionArray.detections,
            self.vision_label)

        vision_result = getNMostProbable(detections, 2, thresh=0.5)

        currentYaw = 0
        if search_direction == 'right':
            currentYaw = -self.yaw
        else:
            currentYaw = self.yaw
        c.yawLeftRelative(currentYaw)

        if len(vision_result) == 0:
            print(search_direction)
            print("yaw: {}".format(currentYaw))
            c.publish()
            self._poll_rate.sleep()
            self.exit('none')
        elif len(vision_result) == 1:
            self.exit('1 post')
        elif len(vision_result) == 2:
            self.exit('2 posts')


class flip(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        global search_direction
        if search_direction == 'right':
            search_direction = 'left'
        else:
            search_direction = 'right'
        return 'success'

class Search(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success'])

        with self:
            smach.StateMachine.add("LOST", lost('start_gate'),
                transitions={'1 post': 'SEARCH', 'none':'LOST',
                '2 posts': 'success'})

            smach.StateMachine.add("FLIP", flip(), transitions={'success': 'LOST'})

            smach.StateMachine.add("SEARCH", search('start_gate'),
                transitions={'2 posts': 'success', 'none':'FLIP',
                '1 post': 'SEARCH'})

#  State for searching gate posts
class search(SubscribeState):
    def __init__(self, vision_label, poll_rate=10):
        SubscribeState.__init__(self, "vision", DetectionArray, self.callback,
            outcomes=['2 posts', 'none', '1 post'])
        self.vision_label = vision_label
        self.yaw = rospy.get_param("ai/search_gate/yaw_speed_factor")
        self._poll_rate = rospy.Rate(poll_rate)

    def callback(self, detectionArray):
        global search_direction
        c = control_wrapper()
        c.forwardError(0.0)

        detections = filterByLabel(detectionArray.detections,
            self.vision_label)
        vision_result = getNMostProbable(detections, 2, thresh=0.5)

        print("results: {}".format(len(vision_result)))

        currentYaw = 0
        if search_direction == 'right':
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
            print(search_direction)
            self.exit('none')

# State for centering between gate posts or moving while being centered
class center(SubscribeState):
    def __init__(self, vision_label):
        SubscribeState.__init__(self, "vision", DetectionArray, self.callback,
            outcomes=['centered', 'lost'])
        self.vision_label = vision_label
        self.errorGoal = rospy.get_param("ai/center/errorGoal")
        self.yaw_factor = rospy.get_param("ai/center/yaw_factor")
        self.dive_factor = rospy.get_param("ai/center/dive_factor")


    def callback(self, detectionArray):
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

        print("Gate X: {}\tGate Y: {}".format(gateXPos, gateYPos))

        if abs(gateXPos-0.5) > self.errorGoal:
            # If we are not centered by yaw
            yaw_left = (gateXPos-0.5) * self.yaw_factor
            print("trying to yaw: {}".format(yaw_left))
            c.yawLeftRelative(yaw_left * 6)
        elif abs(gateYPos-0.5) > self.errorGoal:
            # If our depth is not enough for centering
            dive = (gateYPos-0.5) * self.dive_factor
            print("trying to dive: {}".format(dive))
            c.diveRelative(dive)
        else:
            self.exit('centered')

        c.publish()

# State for moving forward while checking if we are centered
class move_forward_centered(SubscribeState):
    def __init__(self, vision_label):
        SubscribeState.__init__(self, "vision", DetectionArray, self.callback,
        outcomes=['ready', 'not centered', 'lost'])
        self.vision_label = vision_label
        self.distanceGoal = rospy.get_param("ai/move_forward/distanceGoal")
        self.errorGoal = rospy.get_param("ai/move_forward/errorGoal")
        self.forward_speed = rospy.get_param("ai/move_forward/forward_speed")

    def callback(self, detectionArray):
        c = control_wrapper()
        c.levelOut()
        c.forwardError(self.forward_speed)

        detections = filterByLabel(detectionArray.detections,
            self.vision_label)
        vision_result = getNMostProbable(detections, 2, thresh=0.5)

        gateXPos = (vision_result[0].x + vision_result[1].x) / 2
        gateYPos = (vision_result[0].y + vision_result[1].y) / 2

        if (((vision_result[0].width * vision_result[0].height) +
           (vision_result[1].width * vision_result[1].height)) > self.distanceGoal):
            self.exit('ready')
            return 'ready'
        if abs(gateXPos) > self.errorGoal or abs(gateYPos) > self.errorGoal:
            self.exit('not centered')
        if len(vision_result) < 2:
            self.exit('lost')
        c.publish()


# TODO High state machine of gate task
class gate_task(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success'])
        self.time = rospy.get_param("ai/gate_task/forward_time")
        self.speed = rospy.get_param("ai/gate_task/forward_speed")

        with self:
            smach.StateMachine.add('FORWARD_UNTIL_FOUND_GATE',
                move_to_gate('start_gate'), transitions={'success': 'CENTER'})

            smach.StateMachine.add('CENTER', center('start_gate'),
                transitions={'centered': 'FORWARD','lost': 'SEARCH_MACHINE'})

            smach.StateMachine.add('SEARCH_MACHINE', Search(),
                transitions={'success': 'CENTER'})

            smach.StateMachine.add('FORWARD',
                move_forward_centered('start_gate'),
                transitions={'ready': 'BLIND_FORWARD', 'not centered': 'CENTER',
                    'lost': 'SEARCH_MACHINE'})


            smach.StateMachine.add('BLIND_FORWARD',
                move_forward(self.time, self.speed),
                transitions={'success': 'success'})

if __name__ == '__main__':
    rospy.init_node('gate_task')
    sm = gate_task()

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcomes = sm.execute()
    sis.stop()
