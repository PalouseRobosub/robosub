#!/usr/bin/python
import rospy
import blind_movement
import gate_states
from start_switch import start_switch
import smach
import smach_ros

# High level state machine of gate task
class gate_task(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success'])
        self.time = rospy.get_param("ai/gate_task/forward_time_seconds")
        self.speed = rospy.get_param("ai/gate_task/forward_speed")

        with self:
            smach.StateMachine.add('FORWARD_UNTIL_FOUND_GATE',
                                  gate_states.move_to_gate('gate_post'),
                                  transitions={'success': 'CENTER'})

            smach.StateMachine.add('CENTER', gate_states.center('gate_post'),
                                  transitions={'centered': 'FORWARD',
                                              'lost': 'SEARCH_FOR_GATES'})

            smach.StateMachine.add('SEARCH_FOR_GATES',
                                  gate_states.Search_for_gates('gate_post'),
                                  transitions={'success': 'CENTER'})

            smach.StateMachine.add('FORWARD',
                                  gate_states.move_forward_centered(
                                                                   'gate_post'),
                                  transitions={'ready': 'BLIND_FORWARD',
                                              'not centered': 'CENTER',
                                              'lost': 'SEARCH_FOR_GATES'})


            smach.StateMachine.add('BLIND_FORWARD',
                                  blind_movement.move_forward(self.time,
                                  self.speed),
                                  transitions={'success': 'success'})

if __name__ == '__main__':
    # To see debug messages add log_level=rospy.DEBUG argument to init_node
    rospy.init_node('ai')

    while rospy.get_time() == 0:
        continue

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
