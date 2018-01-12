#!/usr/bin/python
import rospy
from blind_movement import move_forward
from gate_util import *
from start_switch import start_switch
import smach
import smach_ros

# High level state machine of navigation channel
class nav_channel(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success'])
        self.time = rospy.get_param("ai/gate_task/forward_time")
        self.speed = rospy.get_param("ai/gate_task/forward_speed")

        with self:
            smach.StateMachine.add('FORWARD_UNTIL_FOUND_GATE',
                                  move_to_gate('nav_channel_post'),
                                  transitions={'success': 'CENTER'})

            smach.StateMachine.add('CENTER', center('nav_channel_post'),
                                  transitions={'centered': 'FORWARD',
                                              'lost': 'SEARCH_FOR_POSTS'})

            smach.StateMachine.add('SEARCH_FOR_POSTS',
                                  Search_for_gates('nav_channel_post'),
                                  transitions={'success': 'CENTER'})

            smach.StateMachine.add('FORWARD',
                                  move_forward_centered('nav_channel_post'),
                                  transitions={'ready': 'BLIND_FORWARD',
                                              'not centered': 'CENTER',
                                              'lost': 'SEARCH_FOR_POSTS'})

            smach.StateMachine.add('BLIND_FORWARD',
                                  move_forward(self.time, self.speed),
                                  transitions={'success': 'success'})

if __name__ == '__main__':
    # To see debug messages add log_level=rospy.DEBUG argument to init_node
    rospy.init_node('ai')

    while rospy.get_time() == 0:
        continue

    sm = smach.StateMachine(outcomes=['success'])
    with sm:
        smach.StateMachine.add('START_SWITCH', start_switch(),
                              transitions={'success': 'NAV_TASK'})
        smach.StateMachine.add('NAV_TASK', nav_channel(),
                              transitions={'success': 'success'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcomes = sm.execute()
    sis.stop()
