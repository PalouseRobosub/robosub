#!/usr/bin/python
import sys
import rospy
from blind_movement import move_forward
from gate_util import *
from start_switch import start_switch
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

            smach.StateMachine.add('SEARCH_FOR_GATES',
                                  Search_for_gates('gate_post'),
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
    # To see debug messages use --debug flag
    if len(sys.argv) > 1:
        if sys.argv[1] == "debug" or sys.argv[1] == "--debug":
            rospy.init_node('ai', log_level=rospy.DEBUG)
        else:
            print("usage:\nrosrun smach_gate.py")
            print("rosrun smach_gate.py --debug for debug mode")
            exit(1)
    else:
        rospy.init_node('ai')
    sm = smach.StateMachine(outcomes=['success'])
    with sm:
        smach.StateMachine.add('START_SWITCH', start_switch(),
                              transitions={'success': 'GATE_TASK'})
        smach.StateMachine.add('GATE_TASK', gate_task(),
                              transitions={'success': 'success'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcomes = sm.execute()
    sis.stop()
