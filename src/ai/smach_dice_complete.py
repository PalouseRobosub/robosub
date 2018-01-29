# uncompyle6 version 2.15.0
# Python bytecode 2.7 (62211)
# Decompiled from: Python 2.7.12 (default, Nov 20 2017, 18:23:56) 
# [GCC 5.4.0 20160609]
# Embedded file name: /home/ryan/repositories/robosub/src/ai/smach_dice_complete.py
# Compiled at: 2018-01-13 14:33:33
import basic_states, control_wrapper, numpy as np, rospy, smach, smach_ros, tf
from SubscribeState import SubscribeState
from robosub.msg import ObstaclePosArray

class GetTargetAngle(SubscribeState):
    """Finds the closest dice target and determines the best angle of approach.

    Attributes:
        yaw: The submarine's current yaw.
    """

    def __init__(self, max_duration=10):
        """Initializes the state.

        Args:
            max_duration: The maximum duration the task may run for in seconds.
        """
        SubscribeState.__init__(self, 'vision/relative', ObstaclePosArray, self.vision_callback, outcomes=[
         'success', 'fail'], input_keys=[
         'target'], output_keys=[
         'yaw'], timeout=max_duration)

    def vision_callback(self, obstacle_position_msg, user_data):
        """ROS callback for relative obstacle positions."""
        dice_detections = [ obstacle for obstacle in obstacle_position_msg.data if obstacle.name.endswith('_dice')
                          ]
        distances = []
        for i, detection in enumerate(dice_detections):
            if detection.x > 0:
                distances.append((i, detection.x))

        if len(distances) < 2:
            return
        sorted_distances = sorted(distances, key=lambda tup: tup[1])
        closest_labels = [
         dice_detections[sorted_distances[0][0]].name,
         dice_detections[sorted_distances[1][0]].name]
        intersect = list(set(closest_labels).intersection([user_data.target]))
        if len(intersect) < 1:
            self.exit('fail')
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
        user_data.yaw = yaw_left
        rospy.loginfo(('Yawing {} to approach angle').format(yaw_left))
        self.exit('success')


class CenterTarget(SubscribeState):

    def __init__(self, distance, max_duration=45):
        SubscribeState.__init__(self, 'vision/relative', ObstaclePosArray, self.vision_callback, outcomes=[
         'success', 'fail'], input_keys=[
         'target'], output_keys=[
         'new_targets'], timeout=max_duration)
        self.max_depth_difference = 0.1
        self.max_position_difference = 0.1
        self.distance = distance

    def vision_callback(self, obstacle_position_msg, user_data):
        """ROS callback for relative obstacle positions."""
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
        z_good = abs(target.z) <= self.max_depth_difference
        y_good = abs(target.y) <= self.max_position_difference
        x_good = abs(target.x - self.distance) <= self.max_position_difference
        if not z_good:
            rospy.logdebug(('Centering Depth ({})').format(target.z))
            c.diveRelative(target.z)
        if not y_good:
            rospy.logdebug(('Centering Y ({})').format(target.y))
            c.strafeLeftError(target.y)
        if y_good and z_good and not x_good:
            forward = target.x - self.distance
            rospy.loginfo(('Centering distance... ({})').format(forward))
            c.forwardError(forward)
        c.publish()
        if x_good and y_good and z_good:
            new_targets = self.get_new_targets(user_data.target)
            user_data.new_targets = new_targets
            rospy.loginfo(('Ramming {} Next Targets: {}').format(user_data.target, new_targets))
            self.exit('success')
        return

    def get_new_targets(self, current_target):
        if current_target is 'one_dice':
            return ['six_dice']
        if current_target is 'two_dice':
            return ['five_dice']
        if current_target is 'three_dice':
            return ['four_dice']
        if current_target is 'four_dice':
            return ['three_dice']
        if current_target is 'five_dice':
            return ['two_dice', 'six_dice']
        if current_target is 'six_dice':
            return ['one_dice', 'five_dice']
        return [
         '']


class SetDistance(SubscribeState):

    def __init__(self, distance, error=0.25, speed=1, max_duration=45):
        SubscribeState.__init__(self, 'vision/relative', ObstaclePosArray, self.vision_callback, outcomes=[
         'success', 'fail'], timeout=max_duration)
        self.speed = speed
        self.min_distance = distance - error
        self.max_distance = distance + error

    def vision_callback(self, obstacle_position_msg, user_data):
        """ROS callback for relative obstacle positions."""
        dice_detections = [ obstacle for obstacle in obstacle_position_msg.data if obstacle.name.endswith('_dice')
                          ]
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
        else:
            if closest_distance > self.max_distance:
                c.forwardError(self.speed)
                c.publish()
            else:
                c.publish()
                self.exit('success')


class RamDice(smach.StateMachine):

    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=[
         'hit', 'fail'], input_keys=[
         'target'], output_keys=[
         'options'])
        with self:
            smach.StateMachine.add('SET_DISTANCE', SetDistance(2.0), transitions={'success': 'ANGLE_TARGET','fail': 'fail',
               'timeout': 'fail'
               })
            smach.StateMachine.add('ANGLE_TARGET', GetTargetAngle(), transitions={'success': 'YAW_TO_TARGET','fail': 'fail',
               'timeout': 'fail'
               }, remapping={'target': 'target','yaw': 'target_angle'
               })
            smach.StateMachine.add('YAW_TO_TARGET', basic_states.YawRelative(), transitions={'success': 'CENTER_APPROACH_TARGET','timeout': 'fail'
               }, remapping={'yaw_left': 'target_angle'})
            smach.StateMachine.add('CENTER_APPROACH_TARGET', CenterTarget(1.0), transitions={'success': 'RAM','fail': 'fail',
               'timeout': 'fail'
               }, remapping={'target': 'target','new_targets': 'options'
               })
            smach.StateMachine.add('RAM', basic_states.BlindRam(4.0, ramming_speed=1), transitions={'success': 'REVERSE'})
            smach.StateMachine.add('REVERSE', basic_states.BlindRam(3.0, ramming_speed=-1), transitions={'success': 'hit'})


class CheckClosest(SubscribeState):
    """Finds the closest two dice targets."""

    def __init__(self):
        """Initializes the state."""
        SubscribeState.__init__(self, 'vision/relative', ObstaclePosArray, self.vision_callback, outcomes=[
         'success', 'fail', 'none'], input_keys=[
         'options'], output_keys=[
         'target'])

    def vision_callback(self, obstacle_position_msg, user_data):
        """ROS callback for relative obstacle positions."""
        dice_detections = [ obstacle for obstacle in obstacle_position_msg.data if obstacle.name.endswith('_dice')
                          ]
        distances = []
        for i, detection in enumerate(dice_detections):
            distance = np.sqrt(detection.x ** 2 + detection.y ** 2 + detection.z ** 2)
            distances.append((i, distance))

        if len(distances) < 2:
            rospy.loginfo(('Detected {} dice').format(len(distances)))
            self.exit('fail')
            return
        sorted_distances = sorted(distances, key=lambda tup: tup[1])
        close_labels = [
         dice_detections[sorted_distances[0][0]].name,
         dice_detections[sorted_distances[1][0]].name]
        intersect = list(set(close_labels).intersection(user_data.options))
        if len(intersect) < 1:
            self.exit('none')
        else:
            user_data.target = intersect[0]
            rospy.loginfo(('Target found: {}').format(intersect[0]))
            self.exit('success')


class RotateLeftAround(smach.StateMachine):

    def __init__(self, yaw_angle, strafe_duration):
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'])
        self.userdata.yaw_angle = -1 * yaw_angle
        self.userdata.strafe = 1
        with self:
            smach.StateMachine.add('INITIAL_DEPTH', basic_states.GoToDepth(2.5), transitions={'success': 'YAW','fail': 'fail',
               'timeout': 'fail'
               })
            smach.StateMachine.add('YAW', basic_states.YawRelative(), transitions={'success': 'STRAFE','timeout': 'fail'
               }, remapping={'yaw_left': 'yaw_angle'})
            smach.StateMachine.add('STRAFE', basic_states.Strafe(strafe_duration), transitions={'success': 'SET_DISTANCE'}, remapping={'strafe_left': 'strafe'})
            smach.StateMachine.add('SET_DISTANCE', SetDistance(2.0), transitions={'success': 'success','fail': 'fail',
               'timeout': 'fail'
               })


class DiceTask(smach.StateMachine):

    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'], input_keys=['options'])
        rotation = 35
        strafe_duration = 3.5
        with self:
            smach.StateMachine.add('INITIAL_DEPTH', basic_states.GoToDepth(2.5), transitions={'success': 'INITIAL_DISTANCE','fail': 'fail',
               'timeout': 'fail'
               })
            smach.StateMachine.add('INITIAL_DISTANCE', SetDistance(2.5), transitions={'success': 'CHECK_CLOSEST_FIRST','fail': 'fail',
               'timeout': 'fail'
               })
            smach.StateMachine.add('CHECK_CLOSEST_FIRST', CheckClosest(), transitions={'success': 'RAM_DICE_FIRST','none': 'ROTATE_LEFT_AROUND_FIRST',
               'fail': 'fail'
               }, remapping={'options': 'options','target': 'target'
               })
            smach.StateMachine.add('ROTATE_LEFT_AROUND_FIRST', RotateLeftAround(rotation, strafe_duration), transitions={'success': 'CHECK_CLOSEST_FIRST','fail': 'fail'
               })
            smach.StateMachine.add('RAM_DICE_FIRST', RamDice(), transitions={'hit': 'CHECK_CLOSEST_SECOND','fail': 'ROTATE_LEFT_AROUND_FIRST'
               }, remapping={'target': 'target','next_targets': 'options'
               })
            smach.StateMachine.add('CHECK_CLOSEST_SECOND', CheckClosest(), transitions={'success': 'RAM_DICE_SECOND','none': 'ROTATE_LEFT_AROUND_SECOND',
               'fail': 'fail'
               }, remapping={'options': 'options','target': 'target'
               })
            smach.StateMachine.add('ROTATE_LEFT_AROUND_SECOND', RotateLeftAround(rotation, strafe_duration), transitions={'success': 'CHECK_CLOSEST_SECOND','fail': 'fail'
               })
            smach.StateMachine.add('RAM_DICE_SECOND', RamDice(), transitions={'hit': 'success','fail': 'ROTATE_LEFT_AROUND_SECOND'
               }, remapping={'target': 'target','next_targets': 'options'
               })


if __name__ == '__main__':
    rospy.init_node('ai')
    while rospy.get_time() == 0:
        continue

    sm = smach.StateMachine(outcomes=['success', 'fail'])
    sm.userdata.dice_options = [
     'five_dice', 'six_dice']
    with sm:
        smach.StateMachine.add('DICE', DiceTask(), transitions={'success': 'success','fail': 'fail'
           }, remapping={'options': 'dice_options'})
    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()
