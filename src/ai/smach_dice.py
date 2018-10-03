#!/usr/bin/python
"""
Author: Ryan Summers
Date: 1-29-2018

Description: Complete dice AI for hitting multiple dice in order.
"""

import rospy
import smach
import smach_ros
import basic_states
import dice_states
from start_switch import start_switch

class DiceTask(smach.StateMachine):
    """ Smach state machine for the dice task.

    Inputs:
        options: A list of desired die targets.
    """

    def __init__(self):
        """Initializes the state machine."""
        smach.StateMachine.__init__(self,
                                    outcomes=['success', 'fail'],
                                    input_keys=['options'])
        rotation = 35
        strafe_duration = 3.5

        with self:
            smach.StateMachine.add('INITIAL_DEPTH', basic_states.GoToDepth(2.5),
                    transitions={'success': 'INITIAL_DISTANCE',
                                 'fail': 'fail',
                                 'timeout': 'fail'})

            smach.StateMachine.add('INITIAL_DISTANCE',
                    dice_states.SetDistance(2.5),
                    transitions={'success': 'CHECK_CLOSEST_FIRST',
                                 'fail': 'fail',
                                 'timeout': 'fail'})

            smach.StateMachine.add('CHECK_CLOSEST_FIRST',
                    dice_states.CheckClosest(),
                    transitions={'success': 'RAM_DICE_FIRST',
                                 'none': 'ROTATE_LEFT_AROUND_FIRST',
                                 'fail': 'fail'},
                    remapping={'options': 'options',
                               'target': 'target'})

            smach.StateMachine.add('ROTATE_LEFT_AROUND_FIRST',
                    dice_states.RotateLeftAround(rotation, strafe_duration),
                    transitions={'success': 'CHECK_CLOSEST_FIRST',
                                 'fail': 'fail'})

            smach.StateMachine.add('RAM_DICE_FIRST', dice_states.RamDice(),
                    transitions={'hit': 'CHECK_CLOSEST_SECOND',
                                 'fail': 'ROTATE_LEFT_AROUND_FIRST'},
                    remapping={'target': 'target',
                               'next_targets': 'options'})

            smach.StateMachine.add('CHECK_CLOSEST_SECOND',
                    dice_states.CheckClosest(),
                    transitions={'success': 'RAM_DICE_SECOND',
                                 'none': 'ROTATE_LEFT_AROUND_SECOND',
                                 'fail': 'fail'},
                    remapping={'options': 'options',
                               'target': 'target'})

            smach.StateMachine.add('ROTATE_LEFT_AROUND_SECOND',
                    dice_states.RotateLeftAround(rotation, strafe_duration),
                    transitions={'success': 'CHECK_CLOSEST_SECOND',
                                 'fail': 'fail'})

            smach.StateMachine.add('RAM_DICE_SECOND',
                    dice_states.RamDice(),
                    transitions={'hit': 'success',
                                 'fail': 'ROTATE_LEFT_AROUND_SECOND'},
                    remapping={'target': 'target',
                               'next_targets': 'options'})


if __name__ == '__main__':
    """Main entry point to the application."""
    rospy.init_node('ai')

    # Wait for ros time to become functional.
    while rospy.get_time() == 0:
        continue

    sm = smach.StateMachine(outcomes=['success', 'fail'])

    # The five and six dice give us two potential other dice to add up to 7 or
    # 11.
    sm.userdata.dice_options = ['die_5', 'die_6']

    with sm:
        smach.StateMachine.add('START_SWITCH', start_switch(),
                              transitions={'success': 'DICE'},
                              remapping={'options': 'dice_options'})
        smach.StateMachine.add('DICE', DiceTask(),
                transitions={'success': 'success',
                             'fail': 'fail'},
                remapping={'options': 'dice_options'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()
