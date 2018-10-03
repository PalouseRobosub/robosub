#!/usr/bin/python
import rospy
import smach
import smach_ros
from start_switch import start_switch
import roulette_states
import basic_states

class RouletteTask(smach.StateMachine):
    """Main roulette state task."""

    def __init__(self):
        """Initializes the roulette task."""
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'])

        # Load parameters for timeout in seconds
        self.stabilize_timeout_sec = \
                rospy.get_param("ai/roulette_wheel/stabilize_timeout_sec")
        self.target_color_timeout_sec = \
                rospy.get_param("ai/roulette_wheel/target_color_timeout_sec")
        self.move_to_pinger_timeout_sec = \
                rospy.get_param("ai/roulette_wheel/move_to_pinger_timeout_sec")
        self.depth_timeout_sec = \
                rospy.get_param("ai/roulette_wheel/go_to_depth_timeout_sec")
        self.stabilize_timeout_sec = \
                rospy.get_param("ai/roulette_wheel/stabilize_timeout_sec")

        with self:
            # First, dive to a certain depth and search for the pinger.
            smach.StateMachine.add('DIVE_PINGER', basic_states.GoToDepth(1.0,
                                  max_duration=self.depth_timeout_sec),
                    transitions={'success': 'FIND_PINGER',
                                 'fail': 'fail',
                                 'timeout': 'fail'})

            smach.StateMachine.add('FIND_PINGER', roulette_states.MoveToPinger(
                                  max_duration=self.move_to_pinger_timeout_sec),
                    transitions={'success': 'STABILIZE',
                                 'fail': 'fail',
                                 'timeout': 'fail'})

            smach.StateMachine.add('STABILIZE', basic_states.Stabilize(
                                  max_duration=self.stabilize_timeout_sec),
                    transitions={'success': 'LOCATE_ROULETTE',
                                 'fail': 'fail',
                                 'timeout': 'LOCATE_ROULETTE'})

            # Next, locate the roulette wheel and center above it.
            smach.StateMachine.add('LOCATE_ROULETTE',
                    roulette_states.CenterAbove(),
                    transitions={'success': 'DIVE_TARGET',
                                 'fail': 'fail'})

            # Now, dive lower and center above the desired color.
            smach.StateMachine.add('DIVE_TARGET', basic_states.GoToDepth(3,
                                  max_duration=self.depth_timeout_sec),
                    transitions={'success': 'CENTER_TARGET',
                                 'fail': 'fail',
                                 'timeout': 'fail'})

            smach.StateMachine.add('CENTER_TARGET',
                    roulette_states.TargetColor(roulette_states.Color.GREEN,
                                  max_duration=self.target_color_timeout_sec),
                    transitions={'success': 'DROP_TARGET',
                                 'fail': 'fail',
                                 'timeout': 'DROP_TARGET'})

            # Finally, drop the marker.
            smach.StateMachine.add('DROP_TARGET', basic_states.DropMarker(),
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
