#!/usr/bin/python
"""
Author: Ryan Summers
Date: 1-29-2018

Description: Complete dice AI for hitting multiple dice in order.
"""
import basic_states
import control_wrapper
import numpy as np
import rospy
import smach
import smach_ros
import tf
from start_switch import start_switch

from SubscribeState import SubscribeState
from robosub_msgs.msg import ObstaclePosArray


class GetTargetAngle(SubscribeState):
    """Finds the closest dice target and determines the best angle of approach.

    Inputs:
        target: The potential die targets that can be hit.

    Outputs:
        yaw: The angle of approach for the targetted dice.
    """

    def __init__(self, max_duration=10):
        """Initializes the state.

        Args:
            max_duration: The maximum duration the task may run for in seconds.
        """
        SubscribeState.__init__(self,
                                'vision/relative',
                                ObstaclePosArray,
                                self.vision_callback,
                                outcomes=['success', 'fail'],
                                input_keys=['target'],
                                output_keys=['yaw'],
                                timeout=max_duration)

    def vision_callback(self, obstacle_position_msg, user_data):
        """ROS callback for relative obstacle positions."""
        dice_detections = [obstacle for obstacle in obstacle_position_msg.data
                           if obstacle.name.startswith('die_')]

        distances = []
        for i, detection in enumerate(dice_detections):
            if detection.x > 0:
                distances.append((i, detection.x))

        if len(distances) < 2:
            return

        sorted_distances = sorted(distances, key=lambda tup: tup[1])

        closest_labels = [dice_detections[sorted_distances[0][0]].name,
                          dice_detections[sorted_distances[1][0]].name]

        # Find the target dice in the list of the two closest dice.
        intersect = list(set(closest_labels).intersection([user_data.target]))
        if len(intersect) < 1:
            self.exit('fail')

        # Find the approach angle by finding the normal line to the two closest
        # dice.
        i = sorted_distances[0][0]
        j = sorted_distances[1][0]

        run = dice_detections[i].y - dice_detections[j].y
        rise = dice_detections[j].x - dice_detections[i].x

        yaw_left = np.arctan2(rise, run) * 180 / np.pi

        if abs(yaw_left) > 90:
            if yaw_left < 0:
                yaw_left += 180
            else:
                yaw_left -= 180

        # Log the approach angle as a state output.
        user_data.yaw = yaw_left
        rospy.logdebug('Yawing {} to approach angle'.format(yaw_left))

        self.exit('success')


class CenterTarget(SubscribeState):
    """ Centers the submarine on the desired target for ramming.

    Attributes:
        max_depth_difference: The maximum depth error allowed in targetting (in
            meters).
        max_position_difference = The maximum Y and X positional error allowed
            in targetting (in meters)
        distance: The desired target distance away from the target.

    """

    def __init__(self,
                 distance,
                 max_position_error=0.1,
                 max_depth_error=0.1,
                 max_duration=45):
        """Initializes the SMACH state.

        Args:
            distance: The distance in meters to center away from the target.
            max_duration: The maximum duration of the state in seconds.
            max_position_error: The maximum permissible positional error for
                targetting in meters.
            max_depth_error: The maximum permissible depth error for targetting
                in meters.
        """
        SubscribeState.__init__(self,
                                'vision/relative',
                                ObstaclePosArray,
                                self.vision_callback,
                                outcomes=['success', 'fail'],
                                input_keys=['target'],
                                output_keys=['new_targets'],
                                timeout=max_duration)
        self.max_depth_difference = max_depth_error
        self.max_position_difference = max_position_error
        self.distance = distance


    def vision_callback(self, obstacle_position_msg, user_data):
        """ROS callback for relative obstacle positions."""

        # Find the target in the list of detections.
        target = None
        for detection in obstacle_position_msg.data:
            if detection.name == user_data.target:
                target = detection
                break

        if target is None:
            return

        c = control_wrapper.control_wrapper()
        c.levelOut()
        c.forwardError(0)
        c.strafeLeftError(0)
        c.diveError(0)

        # Determine which axes need correction.
        z_good = abs(target.z) <= self.max_depth_difference
        y_good = abs(target.y) <= self.max_position_difference
        x_good = abs(target.x - self.distance) <= self.max_position_difference
        if not z_good:
            rospy.logdebug('Centering Depth ({})'.format(target.z))
            c.diveRelative(target.z)

        if not y_good:
            rospy.logdebug('Centering Y ({})'.format(target.y))
            c.strafeLeftError(target.y)

        if y_good and z_good and not x_good:
            forward = target.x - self.distance
            rospy.logdebug('Centering distance... ({})'.format(forward))
            c.forwardError(forward)

        c.publish()

        if x_good and y_good and z_good:
            new_targets = self.get_new_targets(user_data.target)
            user_data.new_targets = new_targets
            rospy.loginfo('Ramming {} -> Next Targets: {}'.format(
                        user_data.target, new_targets))
            self.exit('success')


    def get_new_targets(self, current_target):
        """Given the current target, calculates the next die to hit.

        Args:
            current_target: The currently targetted dice.

        Returns:
            The name of the next dice target.
        """
        if current_target is 'die_1':
            return ['die_6']
        elif current_target is 'die_2':
            return ['die_5']
        elif current_target is 'die_3':
            return ['die_4']
        elif current_target is 'die_4':
            return ['die_3']
        elif current_target is 'die_5':
            return ['die_2', 'die_6']
        elif current_target is 'die_6':
            return ['die_1', 'die_5']

        return ['']


class SetDistance(SubscribeState):
    """Sets the distance to any potential dice to a specified value.

    Attributes:
        speed: The speed at which to move
        min_distance: The minimum distance to maintain in meters.
        max_distance: The maximum distance to maintain in meters.
    """

    def __init__(self, distance, error=0.25, speed=1, max_duration=45):
        """Initializes the distance state.

        Args:
            distance: The distance to approach to in meters.
            error: The maximum error off nominal distance in meters.
            speed: The desired correction movement speed.
            max_duration: The maximum duration of the state in seconds.
        """
        SubscribeState.__init__(self,
                                'vision/relative',
                                ObstaclePosArray,
                                self.vision_callback,
                                outcomes=['success', 'fail'],
                                timeout=max_duration)
        self.speed = speed
        self.min_distance = distance - error
        self.max_distance = distance + error


    def vision_callback(self, obstacle_position_msg, user_data):
        """ROS callback for relative obstacle positions."""
        dice_detections = [obstacle for obstacle in obstacle_position_msg.data
                           if obstacle.name.startswith('die_')]

        if len(dice_detections) == 0:
            rospy.loginfo('No dice detected.')
            self.exit('fail')
            return

        # Find the closest dice position.
        closest_distance = 5000
        for detection in dice_detections:
            if detection.x > 0 and detection.x < closest_distance:
                closest_distance = detection.x

        c = control_wrapper.control_wrapper()
        c.levelOut()
        c.strafeLeftError(0)
        c.forwardError(0)

        if closest_distance < self.min_distance:
            c.forwardError(-1 * self.speed)
            c.publish()
        elif closest_distance > self.max_distance:
            c.forwardError(self.speed)
            c.publish()
        else:
            c.publish()
            self.exit('success')


class RamDice(smach.StateMachine):
    """ State machine to ram a dice."""

    def __init__(self):
        smach.StateMachine.__init__(self,
                                    outcomes=['hit', 'fail'],
                                    input_keys=['target'],
                                    output_keys=['options'])

        with self:
            smach.StateMachine.add('SET_DISTANCE', SetDistance(2.0),
                    transitions={'success': 'ANGLE_TARGET',
                                 'fail': 'fail',
                                 'timeout': 'fail'})

            smach.StateMachine.add('ANGLE_TARGET', GetTargetAngle(),
                    transitions={'success': 'YAW_TO_TARGET',
                                 'fail': 'fail',
                                 'timeout': 'fail'},
                    remapping={'target': 'target',
                               'yaw': 'target_angle'})

            smach.StateMachine.add('YAW_TO_TARGET', basic_states.YawRelative(),
                    transitions={'success': 'CENTER_APPROACH_TARGET',
                                 'timeout': 'fail'},
                    remapping={'yaw_left': 'target_angle'})

            smach.StateMachine.add('CENTER_APPROACH_TARGET', CenterTarget(1.0),
                    transitions={'success': 'RAM',
                                 'fail': 'fail',
                                 'timeout': 'fail'},
                    remapping={'target': 'target',
                               'new_targets': 'options'})

            smach.StateMachine.add('RAM',
                    basic_states.BlindRam(4.0, ramming_speed=1),
                    transitions={'success': 'REVERSE'})

            smach.StateMachine.add('REVERSE',
                    basic_states.BlindRam(3.0, ramming_speed=-1),
                    transitions={'success': 'hit'})


class CheckClosest(SubscribeState):
    """Finds the closest two dice targets.

    Inputs:
        options: A list of potential vision labels to detect.

    Outputs:
        target: The closest target from the available options.
    """

    def __init__(self):
        """Initializes the state."""
        SubscribeState.__init__(self,
                                'vision/relative',
                                ObstaclePosArray,
                                self.vision_callback,
                                outcomes=['success', 'fail', 'none'],
                                input_keys=['options'],
                                output_keys=['target'])


    def vision_callback(self, obstacle_position_msg, user_data):
        """ROS callback for relative obstacle positions."""
        dice_detections = [obstacle for obstacle in obstacle_position_msg.data
                           if obstacle.name.startswith('die_')]

        # Find the distances of all of the dice.
        distances = []
        for i, detection in enumerate(dice_detections):
            distance = np.sqrt(detection.x ** 2 +
                               detection.y ** 2 +
                               detection.z ** 2)
            distances.append((i, distance))

        if len(distances) < 2:
            rospy.logdebug('Detected {} dice'.format(len(distances)))
            self.exit('fail')
            return

        # Find the two closest labels.
        sorted_distances = sorted(distances, key=lambda tup: tup[1])
        close_labels = [dice_detections[sorted_distances[0][0]].name,
                        dice_detections[sorted_distances[1][0]].name]

        # Check if any of the closest labels are any of the potential options.
        intersect = list(set(close_labels).intersection(user_data.options))

        if len(intersect) < 1:
            self.exit('none')
        else:
            user_data.target = intersect[0]
            rospy.loginfo('Target found: {}'.format(intersect[0]))
            self.exit('success')


class RotateLeftAround(smach.StateMachine):
    """ State machine that rotates left around an object."""


    def __init__(self, yaw_angle, strafe_duration):
        """Initializes the state machine.

        Args:
            yaw_angle: The angle to in degrees to yaw before strafing.
            strafe_duration: The duration of a strafe following a yaw in
                seconds.
        """
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'])

        self.userdata.yaw_angle = -1 * abs(yaw_angle)
        self.userdata.strafe = 1

        with self:
            smach.StateMachine.add('INITIAL_DEPTH', basic_states.GoToDepth(2.5),
                    transitions={'success': 'YAW',
                                 'fail': 'fail',
                                 'timeout': 'fail'})

            smach.StateMachine.add('YAW', basic_states.YawRelative(),
                    transitions={'success': 'STRAFE',
                                 'timeout': 'fail'},
                    remapping={'yaw_left': 'yaw_angle'})

            smach.StateMachine.add('STRAFE',
                    basic_states.BlindStrafe(strafe_duration),
                    transitions={'success': 'SET_DISTANCE'},
                    remapping={'strafe_left': 'strafe'})

            smach.StateMachine.add('SET_DISTANCE', SetDistance(2.0),
                    transitions={'success': 'success',
                                 'fail': 'fail',
                                 'timeout': 'fail'})


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

            smach.StateMachine.add('INITIAL_DISTANCE', SetDistance(2.5),
                    transitions={'success': 'CHECK_CLOSEST_FIRST',
                                 'fail': 'fail',
                                 'timeout': 'fail'})

            smach.StateMachine.add('CHECK_CLOSEST_FIRST', CheckClosest(),
                    transitions={'success': 'RAM_DICE_FIRST',
                                 'none': 'ROTATE_LEFT_AROUND_FIRST',
                                 'fail': 'fail'},
                    remapping={'options': 'options',
                               'target': 'target'})

            smach.StateMachine.add('ROTATE_LEFT_AROUND_FIRST',
                    RotateLeftAround(rotation, strafe_duration),
                    transitions={'success': 'CHECK_CLOSEST_FIRST',
                                 'fail': 'fail'})

            smach.StateMachine.add('RAM_DICE_FIRST', RamDice(),
                    transitions={'hit': 'CHECK_CLOSEST_SECOND',
                                 'fail': 'ROTATE_LEFT_AROUND_FIRST'},
                    remapping={'target': 'target',
                               'next_targets': 'options'})

            smach.StateMachine.add('CHECK_CLOSEST_SECOND', CheckClosest(),
                    transitions={'success': 'RAM_DICE_SECOND',
                                 'none': 'ROTATE_LEFT_AROUND_SECOND',
                                 'fail': 'fail'},
                    remapping={'options': 'options',
                               'target': 'target'})

            smach.StateMachine.add('ROTATE_LEFT_AROUND_SECOND',
                    RotateLeftAround(rotation, strafe_duration),
                    transitions={'success': 'CHECK_CLOSEST_SECOND',
                                 'fail': 'fail'})

            smach.StateMachine.add('RAM_DICE_SECOND', RamDice(),
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
