#!/usr/bin/python
import rospy
import smach
import smach_ros
import buoy_states
from start_switch import start_switch
import blind_movement

class buoy_task(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success'])
        reset_time = rospy.get_param("ai/hit_buoy/reset_time_seconds")
        reset_speed = rospy.get_param("ai/hit_buoy/reset_speed")
        with self:

            smach.StateMachine.add('HIT_BUOY_RED',
                                  buoy_states.hit_buoy('red_buoy'),
                                  transitions={'success': 'RESET_FOR_GREEN'})

            smach.StateMachine.add('RESET_FOR_GREEN',
                                  blind_movement.move_forward(time=reset_time,
                                  value=reset_speed),
                                  transitions={'success': 'HIT_BUOY_GREEN'})

            smach.StateMachine.add('HIT_BUOY_GREEN',
                                  buoy_states.hit_buoy('green_buoy'),
                                  transitions={'success': 'RESET_FOR_YELLOW'})

            smach.StateMachine.add('RESET_FOR_YELLOW',
                                  blind_movement.move_forward(time=reset_time,
                                  value=reset_speed),
                                  transitions={'success': 'HIT_BUOY_YELLOW'})

            smach.StateMachine.add('HIT_BUOY_YELLOW',
                                  buoy_states.hit_buoy('yellow_buoy'),
                                  transitions={'success': 'BACKUP'})

            smach.StateMachine.add('BACKUP',
                                  blind_movement.move_forward(time=reset_time,
                                  value=reset_speed),
                                  transitions={'success': 'success'})

if __name__ == "__main__":
    # To see debug messages add log_level=rospy.DEBUG argument to init_node
    rospy.init_node('ai')

    while rospy.get_time() == 0:
        continue

    sm = smach.StateMachine(outcomes=['success'])
    with sm:
        smach.StateMachine.add('START_SWITCH', start_switch(),
                              transitions={'success': 'BUOY_TASK'})
        smach.StateMachine.add('BUOY_TASK', buoy_task(),
                              transitions={'success': 'success'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()

    sis.stop()
