#!/usr/bin/python
import rospy
from blind_movement import move_forward
from util_states import *
import smach
import smach_ros

# High level state machine of gate task
class gate_task(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success'])
        self.time = rospy.get_param("ai/gate_task/forward_time")
        self.speed = rospy.get_param("ai/gate_task/forward_speed")

        with self:
            smach.StateMachine.add('FORWARD_UNTIL_FOUND_GATE',
                                  move_to_gate('gate_post'),
                                  transitions={'success': 'CENTER'})

            smach.StateMachine.add('CENTER', center('gate_post'),
                                  transitions={'centered': 'FORWARD',
                                  'lost': 'SEARCH_FOR_GATES'})

            smach.StateMachine.add('SEARCH_FOR_GATES', Search_for_gates(),
                                  transitions={'success': 'CENTER'})

            smach.StateMachine.add('FORWARD',
                                  move_forward_centered('gate_post'),
                                  transitions={'ready': 'BLIND_FORWARD',
                                  'not centered': 'CENTER',
                                  'lost': 'SEARCH_FOR_GATES'})


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
