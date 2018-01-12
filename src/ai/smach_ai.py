#!/usr/bin/python
import rospy
from start_switch import start_switch
from smach_gate import gate_task
from smach_nav_channel import nav_channel
from smach_buoy import buoy_task
import smach
import smach_ros

# High state machine of AI that contains Gate task, Bouy task, and Nav Channel
class MAIN_AI(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success'])

        with self:
            smach.StateMachine.add('GATE_TASK', gate_task(),
                                  transitions={'success': 'BOUY_TASK'})
            smach.StateMachine.add('BOUY_TASK', buoy_task(),
                                  transitions={'success': 'NAV_TASK'})
            smach.StateMachine.add('NAV_TASK', nav_channel(),
                                  transitions={'success': 'success'})

if __name__ == '__main__':

    while rospy.get_time() == 0
        continue

    # To see debug messages add log_level=rospy.DEBUG argument to init_node
    rospy.init_node('ai')
    sm = smach.StateMachine(outcomes=['success'])
    with sm:
        smach.StateMachine.add('START_SWITCH', start_switch(),
                              transitions={'success': 'MAIN_AI'})
        smach.StateMachine.add('MAIN_AI', MAIN_AI(),
                              transitions={'success': 'success'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcomes = sm.execute()
    sis.stop()
