#!/usr/bin/python
import rospy
from blind_movement import move_forward
from gate_util import *
from start_switch import start_switch
import smach
import smach_ros
from basic_states import *

class blind_gate_task(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success'])

        # these variables will not be found without using rosparam load ai.yaml
        self.time = rospy.get_param("ai/blind_gate_task/forward_time")
        self.speed = rospy.get_param("ai/blind_gate_task/forward_speed")

        with self:
            smach.StateMachine.add('TAKE_HEADING', take_heading_yaw(),
                                   transitions={'success': 'START_SWITCH'},
                                   remapping={'heading_output': 'heading'})

            smach.StateMachine.add('START_SWITCH', start_switch(),
                                   transitions={'success': 'ROTATE_TO_HEADING'})

            smach.StateMachine.add('ROTATE_TO_HEADING', rotate_to_heading(),
                                   transitions={'success': 'MOVE_FORWARD'},
                                   remapping={'heading_input': 'heading'})

            smach.StateMachine.add('MOVE_FORWARD',
                                   move_forward(self.time, self.speed),
                                   transitions={'success': 'success'})

if __name__ == '__main__':
    rospy.init_node('ai')

    while rospy.get_time() == 0:
        continue

    sm = smach.StateMachine(outcomes=['success'])
    with sm:
        smach.StateMachine.add('BLIND_GATE_TASK', blind_gate_task(),
                               transitions={'success': 'success'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcomes = sm.execute()
    sis.stop()
