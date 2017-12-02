#!/usr/bin/python
import rospy
import sys
from blind_movement import move_forward
from gate_util import *
from smach_gate import gate_task
from start_switch import start_switch
import smach
import smach_ros

# High level state machine of gate task
class PreQual_task(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success'])
        self.time = rospy.get_param("ai/preq_forward/forward_time")
        self.speed = rospy.get_param("ai/preq_forward/forward_speed")

        with self:
            smach.StateMachine.add('GATE_TASK', gate_task(),
                                  transitions={'success': 'STRAFE'})
            smach.StateMachine.add('STRAFE', strafe(),
                                  transitions={'success': 'BLIND_FORWARD_POST'})
            smach.StateMachine.add('BLIND_FORWARD_POST',
                                  move_forward(self.time, self.speed),
                                  transitions={'success': 'EXPERIMENT'})
            smach.StateMachine.add('EXPERIMENT', experiment(),
                                  transitions={'success': 'GATE_TASK_BACK'})
            smach.StateMachine.add('GATE_TASK_BACK', gate_task(),
                                  transitions={'success': 'success'})

if __name__ == '__main__':
    # To see debug messages use --debug flag
    if len(sys.argv) > 1:
        if sys.argv[1] == "debug" or sys.argv[1] == "--debug":
            rospy.init_node('ai', log_level=rospy.DEBUG)
        else:
            print("usage:\nrosrun smach_QualGate.py")
            print("rosrun smach_QualGate.py --debug for debug mode")
            exit(1)
    else:
        rospy.init_node('ai')
    sm = smach.StateMachine(outcomes=['success'])
    with sm:
        smach.StateMachine.add('START_SWITCH', start_switch(),
                              transitions={'success': 'PREQUAL_TASK'})
        smach.StateMachine.add('PREQUAL_TASK', PreQual_task(),
                              transitions={'success': 'success'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcomes = sm.execute()
    sis.stop()
