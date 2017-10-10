#!/usr/bin/python
import rospy
from blind_movement import move_forward
from util_states import *
import smach
import smach_ros

# High level state machine for searching navigation channel posts
class Search_for_posts(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success'])

        with self:
            smach.StateMachine.add("LOST", lost('nav_channel_post'),
                transitions={'1 post': 'SEARCH', 'none': 'LOST',
                '2 posts': 'SEARCH'})

            smach.StateMachine.add("FLIP", flip(),
                                  transitions={'success': 'LOST'})

            smach.StateMachine.add("SEARCH", search('nav_channel_post'),
                transitions={'2 posts': 'success', 'none': 'FLIP',
                '1 post': 'SEARCH'})

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
                                  'lost': 'SEARCH_FOR_GATES'})

            smach.StateMachine.add('SEARCH_FOR_POSTS', Search_for_posts(),
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
    rospy.init_node('nav_channel')
    sm = nav_channel()

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcomes = sm.execute()
    sis.stop()
