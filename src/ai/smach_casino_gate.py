"""
 @author Ryan Summers
 @date 1-29-2018

 @brief Smach-based AI for going through the starting Casino gate.
"""
import basic_states
import control_wrapper
import numpy as np
import rospy
import smach
import smach_ros

from robosub.msg import ObstaclePosArray
from rs_yolo.msg import DetectionArray
from SubscribeState import SubscribeState


class YawToGate(SubscribeState):
    """Yaws toward the gate until both posts are in view.

    Attributes:
        yaw_speed: The speed at which the submarine yaws left.
        retry_count: The maximum number of messages that do not contain a gate
            post.
        max_duration: The maximum duration of the state in seconds.
    """

    def __init__(self, yaw_speed=-20, retry_count=5, max_duration=30):
        """Initializes the SMACH state.

        Args:
            yaw_speed: The speed to yaw left at in search of the gate.
            retry_count: The maximum number of messages that do not contain a
                single gate post that may occur.
            max_duration: The maximum duration of the state in seconds.
        """
        SubscribeState.__init__(self,
                                'vision/left',
                                DetectionArray,
                                self.detection_callback,
                                outcomes=[ 'success', 'fail'],
                                timeout=max_duration)
        self.yaw_speed = yaw_speed
        self.tries = 0
        self.retry_count = retry_count


    def detection_callback(self, detection_msg, user_data):
        """ROS callback for vision detections."""
        number_posts = len([detection for detection in detection_msg.detections
                            if detection.label == 'start_gate_post'])

        c = control_wrapper.control_wrapper()
        c.levelOut()
        c.forwardError(0)
        c.strafeLeftError(0)
        c.yawLeftError(0)
        rospy.logdebug('Found {} start gate posts.'.format(number_posts))

        # If both poles have been found, stop moving and return success.
        if number_posts == 2:
            c.publish()
            self.exit('success')
            return

        # Otherwise, continue searching for both poles.
        c.yawLeftRelative(self.yaw_speed)
        c.publish()

        if number_posts == 0:
            if self.tries >= self.retry_count:
                self.exit('fail')
            self.tries = self.tries + 1


class FindGate(smach.StateMachine):
    """Finds the gate and yaws toward it."""

    def __init__(self):
        """ Initializes the SMACH state machine."""
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'])

        with self:
            smach.StateMachine.add('LOCATE_POLE',
                    basic_states.LocateObject('start_gate_post',
                                              yaw_speed=-20),
                            transitions={'success': 'YAW_TO_GATE',
                                         'fail': 'fail',
                                         'timeout': 'fail'})

            smach.StateMachine.add('YAW_TO_GATE', YawToGate(),
                    transitions={'success': 'success',
                                 'timeout': 'fail',
                                 'fail': 'fail'})


class CenterGate(SubscribeState):
    """ Centers on a gate target side at specified distance.

    Inputs:
        side: The target side to center on. Can be 'left'/'black' or
            'right'/'red'.

    Attributes:
        max_horizontal_error: The maximum horizontal distance off the optimal
            center allowed in meters.
        max_distance_error: The maximum error in distance from the target
            distance allowed in meters.
        gate_width: The width of the start gate in meters.
        distance_goal: The target distance away from the gate.
        tries: The number of messages without two gate posts in view.
        retry_count: The maximum number of messages without two gate posts in
            view allowed.
    """

    def __init__(self,
                 distance,
                 retry_count=5,
                 max_duration=45,
                 gate_width=3,
                 max_horizontal_error=0.2,
                 max_distance_error=0.4):
        """ Initializes the SMACH state.

        Args:
            distance: The distance in meters to approach to.
            max_duration: The maximum duration of the state in seconds.
            gate_width: The width of the gate in meters.
            max_horizontal_error: The maximum horizontal distance off the
                optimal center allowed in meters.
            max_distance_error: The maximum error in distance from the target
                distance allowed in meters.
        """
        SubscribeState.__init__(self,
                                'vision/relative',
                                ObstaclePosArray,
                                self.obstacle_callback,
                                input_keys=['side'],
                                outcomes=['success', 'fail'],
                                timeout=max_duration)
        self.max_horizontal_error = max_horizontal_error
        self.max_distance_error = max_distance_error
        self.gate_width = gate_width
        self.distance_goal = distance
        self.tries = 0
        self.retry_count = retry_count


    def obstacle_callback(self, obstacle_msg, user_data):
        """ROS callback for obstacle detections."""
        gate_posts = [obstacle for obstacle in obstacle_msg.data
                      if obstacle.name == 'start_gate_post']

        if len(gate_posts) != 2:
            if self.tries >= self.retry_count:
                self.exit('fail')
            self.tries = self.tries + 1
            return

        # Divide the gate into two segments and target the left or right middle.
        if user_data.side == 'left' or user_data.side == 'black':
            target = -1 * self.gate_width / 4.0
        elif user_data.side == 'right' or user_data.side == 'red':
            target = self.gate_width / 4.0
        else:
            self.exit('fail')

        c = control_wrapper.control_wrapper()
        c.levelOut()
        c.forwardError(0)
        c.strafeLeftError(0)

        y_average = (gate_posts[0].y + gate_posts[1].y) / 2
        y_error = y_average - target

        x_average = (gate_posts[0].x + gate_posts[1].x) / 2
        x_error = x_average - self.distance_goal

        y_good = abs(y_error) <= self.max_horizontal_error
        x_good = abs(x_error) <= self.max_distance_error

        if not y_good:
            rospy.logdebug('Strafe error: {}'.format(y_error))
            c.strafeLeftError(y_error)

        if not x_good:
            rospy.logdebug('Forward error: {}'.format(x_error))
            c.forwardError(x_error)

        c.publish()
        if y_good and x_good:
            self.exit('success')


class GetGateAngle(SubscribeState):
    """ Calculates the static angle of approach to the start gate.

    Attributes:
        retry_count: The number of messages that can come in without two gate
            posts available.
        tries: The number of messages that have arrived without two gate posts
            available.

    Outputs:
        yaw: The relative yaw_left needed for the angle of approach.
    """

    def __init__(self, retry_count=5, max_duration=45):
        """Initializes the SMACH state.

        Args:
            retry_count: The maximum number of messages that can arrive without
                two gate posts available.
            max_duration: The maximum duration of the state in seconds.
        """
        SubscribeState.__init__(self,
                                'vision/relative',
                                ObstaclePosArray,
                                self.obstacle_callback,
                                outcomes=['success', 'fail'],
                                output_keys=['yaw'],
                                timeout=max_duration)
        self.retry_count = retry_count
        self.tries = 0


    def obstacle_callback(self, obstacle_msg, user_data):
        """ROS callback for relative position messages."""
        gate_posts = [obstacle for obstacle in obstacle_msg.data
                      if obstacle.name == 'start_gate_post']

        if len(gate_posts) != 2:
            if self.tries >= self.retry_count:
                self.exit('fail')
            self.tries = self.tries + 1
            return

        run = gate_posts[0].y - gate_posts[1].y
        rise = gate_posts[1].x - gate_posts[0].x
        yaw_left = np.arctan2(rise, run) * 180 / np.pi
        if abs(yaw_left) > 90:
            if yaw_left < 0:
                yaw_left += 180
            else:
                yaw_left -= 180
        rospy.loginfo(('Gate normal is {} degrees left').format(yaw_left))
        user_data.yaw = yaw_left
        self.exit('success')


class TargetGate(smach.StateMachine):
    """SMACH AI that targets the gate and moves to a blind ramming position.

    Inputs:
        target: The target side to go to. Can be 'left' or 'right'.
    """

    def __init__(self):
        """Initializes the SMACH state machine."""
        smach.StateMachine.__init__(self,
                                    outcomes=['success', 'fail'],
                                    input_keys=['target'])

        with self:
            smach.StateMachine.add('CALCULATE_MOVEMENT', GetGateAngle(),
                    transitions={'success': 'YAW_TOWARDS_GATE',
                                 'timeout': 'fail',
                                 'fail': 'fail'},
                                 remapping={'yaw': 'angle'})

            smach.StateMachine.add('YAW_TOWARDS_GATE',
                    basic_states.YawRelative(),
                    transitions={'success': 'CENTER_GATE',
                                 'timeout': 'fail'},
                    remapping={'yaw_left': 'angle'})

            smach.StateMachine.add('CENTER_GATE', CenterGate(2.0),
                    transitions={'success': 'success',
                                 'timeout': 'fail',
                                 'fail': 'fail'},
                    remapping={'side': 'target'})


class GateTask(smach.StateMachine):
    """SMACH AI that goes through the casino gate on a targetted side.

    Inputs:
        target_side: The targetted gate side. Can be 'left' or 'right'.
    """

    def __init__(self):
        """Initializes the gate task."""
        smach.StateMachine.__init__(self,
                                    outcomes=['success', 'fail'],
                                    input_keys=['target_side'])

        with self:
            smach.StateMachine.add('INITIAL_DEPTH', basic_states.GoToDepth(1.0),
                    transitions={'success': 'FIND_GATE',
                                 'timeout': 'fail',
                                 'fail': 'fail'})

            smach.StateMachine.add('FIND_GATE', FindGate(),
                    transitions={'success': 'TARGET_GATE',
                                 'fail': 'fail'})

            smach.StateMachine.add('TARGET_GATE', TargetGate(),
                    transitions={'success': 'BLIND_FORWARD',
                                 'fail': 'BLIND_FORWARD'},
                    remapping={'target': 'target_side'})

            smach.StateMachine.add('BLIND_FORWARD',
                    basic_states.BlindRam(8.0, ramming_speed=1.0),
                    transitions={'success': 'success'})


if __name__ == '__main__':
    """Main entry point into application. """

    rospy.init_node('ai')

    while rospy.get_time() == 0:
        continue

    sm = smach.StateMachine(outcomes=['success', 'fail'])
    sm.userdata.side = 'right'

    with sm:
        smach.StateMachine.add('CASINO_GATE', GateTask(),
                transitions={'success': 'success',
                             'fail': 'fail'},
                remapping={'target_side': 'side'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()
