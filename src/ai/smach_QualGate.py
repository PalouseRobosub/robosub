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
        self.time_forward = rospy.get_param(
                            "ai/preq_forward/forward_time_seconds")
        self.speed_forward = rospy.get_param("ai/preq_forward/forward_speed")

        self.time_strafe = rospy.get_param("ai/strafe/time_seconds")
        self.speed_strafe = rospy.get_param("ai/strafe/speed")

        with self:
            smach.StateMachine.add('GATE_TASK', gate_task(),
                                  transitions={'success': 'FORWARD_UNTIL_SEE'})
            smach.StateMachine.add('FORWARD_UNTIL_SEE',
                                  move_to_gate('gate_post'),
                                  transitions={'success': 'CENTER_SINGLE'})
            smach.StateMachine.add('CENTER_SINGLE', center_object('gate_post'),
                                  transitions={'centered': 'FORWARD_SINGLE',
                                              'lost': 'FORWARD_UNTIL_SEE',
                                              'ready': 'STRAFE'})
            smach.StateMachine.add('FORWARD_SINGLE',
                                  move_forward_centered_single('gate_post'),
                                  transitions={'ready': 'STRAFE',
                                              'not centered': 'CENTER_SINGLE',
                                              'lost': 'FORWARD_UNTIL_SEE'})
            smach.StateMachine.add('STRAFE',
                                  strafe_for_duration(self.time_strafe,
                                  self.speed_strafe),
                                  transitions={'success':
                                  'BLIND_FORWARD_SINGLE'})
            smach.StateMachine.add('BLIND_FORWARD_SINGLE',
                                  move_forward(self.time_forward,
                                  self.speed_forward),
                                  transitions={'success': 'U_TURN'})
            smach.StateMachine.add('U_TURN', u_turn(),
                                  transitions={'success': 'BLIND_FORWARD_BACK'})
            smach.StateMachine.add('BLIND_FORWARD_BACK', move_forward(8, 5),
                                  transitions={'success': 'GATE_TASK_BACK'})
            smach.StateMachine.add('GATE_TASK_BACK', gate_task(),
                                  transitions={'success': 'success'})

if __name__ == '__main__':

    # To see debug messages use _debug=true flag
    debug = rospy.get_param('~debug', default=False)
    if debug:
        rospy.init_node('ai', log_level=rospy.DEBUG)
    else:
        rospy.init_node('ai')
    while rospy.get_time() == 0:
        continue
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
