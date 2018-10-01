#!/usr/bin/python
import rospy
import smach
import smach_ros
from start_switch import start_switch
from roulette_states import *
from basic_states import *

class RouletteTask(smach.StateMachine):
    """Main roulette state task."""

    def __init__(self):
        """Initializes the roulette task."""
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'])

        # Load parameters for timeout in seconds
        self.stabilize_timeout = \
                         rospy.get_param("ai/roulette_wheel/stabilize_timeout")
        self.target_color_timeout = \
                         rospy.get_param(
                         "ai/roulette_wheel/target_color_timeout")
        self.move_to_pinger_timeout = \
                         rospy.get_param(
                         "ai/roulette_wheel/move_to_pinger_timeout")
        self.depth_timeout = \
                         rospy.get_param(
                         "ai/roulette_wheel/go_to_depth_timeout")
        self.stabilize_timeout = \
                         rospy.get_param("ai/roulette_wheel/stabilize_timeout")

        with self:
            # First, dive to a certain depth and search for the pinger.
            smach.StateMachine.add('DIVE_PINGER', GoToDepth(1.0,
                                  max_duration=self.depth_timeout),
                    transitions={'success': 'FIND_PINGER',
                                 'fail': 'fail',
                                 'timeout': 'fail'})

            smach.StateMachine.add('FIND_PINGER', MoveToPinger(
                                  max_duration=self.move_to_pinger_timeout),
                    transitions={'success': 'STABILIZE',
                                 'fail': 'fail',
                                 'timeout': 'fail'})

            smach.StateMachine.add('STABILIZE', Stabilize(
                                  max_duration=self.stabilize_timeout),
                    transitions={'success': 'LOCATE_ROULETTE',
                                 'fail': 'fail',
                                 'timeout': 'LOCATE_ROULETTE'})

            # Next, locate the roulette wheel and center above it.
            smach.StateMachine.add('LOCATE_ROULETTE', CenterAbove(),
                    transitions={'success': 'DIVE_TARGET',
                                 'fail': 'fail'})

            # Now, dive lower and center above the desired color.
            smach.StateMachine.add('DIVE_TARGET', GoToDepth(3,
                                  max_duration=self.depth_timeout),
                    transitions={'success': 'CENTER_TARGET',
                                 'fail': 'fail',
                                 'timeout': 'fail'})

            smach.StateMachine.add('CENTER_TARGET', TargetColor(Color.GREEN,
                                  max_duration=self.target_color_timeout),
                    transitions={'success': 'DROP_TARGET',
                                 'fail': 'fail',
                                 'timeout': 'DROP_TARGET'})

            # Finally, drop the marker.
            smach.StateMachine.add('DROP_TARGET',
                    DropMarker(),
                    transitions={'success': 'success'})

if __name__ == '__main__':
    rospy.init_node('ai')

    # Wait for ROS time to properly begin.
    while rospy.get_time() == 0:
        continue

    sm = smach.StateMachine(outcomes=['success', 'fail'])

    with sm:
        smach.StateMachine.add('START_SWITCH', start_switch(),
                              transitions={'success': 'ROULETTE'})
        smach.StateMachine.add('ROULETTE', RouletteTask(),
                transitions={'success': 'success',
                             'fail': 'fail'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()
