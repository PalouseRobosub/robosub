#!/usr/bin/python
import rospy
import smach
import smach_ros
from start_switch import start_switch
import roulette_states
import basic_states

class surface_ai(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success'])

        with self:
            smach.StateMachine.add('MOVE_TO_PINGER', roulette_states.MoveToPinger(),
                                  transitions={'success': 'SURFACE', 'fail': 'MOVE_TO_PINGER',
                                              'timeout': 'MOVE_TO_PINGER'})
            smach.StateMachine.add('SURFACE', basic_states.surface(), transitions={'success': 'success'})


if __name__ == '__main__':
    rospy.init_node('ai')

    while rospy.get_time() == 0:
        continue


    sm = smach.StateMachine(outcomes=['success'])
    with sm:
        smach.StateMachine.add('START_SWITCH', start_switch(), transitions={'success': 'SURFACE_TASK'})
        smach.StateMachine.add('SURFACE_TASK', surface_ai(), transitions={'success': 'success'})


    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcomes = sm.execute()
    sis.stop()
