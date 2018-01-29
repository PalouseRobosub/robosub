# uncompyle6 version 2.15.0
# Python bytecode 2.7 (62211)
# Decompiled from: Python 2.7.12 (default, Nov 20 2017, 18:23:56) 
# [GCC 5.4.0 20160609]
# Embedded file name: /home/ryan/repositories/robosub/src/ai/smach_casino_gate.py
# Compiled at: 2018-01-13 14:33:33
import basic_states, control_wrapper, numpy as np, rospy, smach, smach_ros, tf
from SubscribeState import SubscribeState
from robosub.msg import ObstaclePosArray
from rs_yolo.msg import DetectionArray

class YawToGate(SubscribeState):

    def __init__(self, yaw_speed=-20, max_duration=30):
        SubscribeState.__init__(self, 'vision/left', DetectionArray, self.detection_callback, outcomes=[
         'success', 'fail'], timeout=max_duration)
        self.yaw_speed = yaw_speed

    def detection_callback(self, detection_msg, user_data):
        detections = detection_msg.detections
        posts = 0
        for detection in detections:
            if detection.label == 'start_gate_post':
                posts = posts + 1

        c = control_wrapper.control_wrapper()
        c.levelOut()
        c.forwardError(0)
        c.strafeLeftError(0)
        c.yawLeftError(0)
        rospy.loginfo(('Found {} start get posts.').format(posts))
        if posts == 1:
            c.yawLeftRelative(self.yaw_speed)
            c.publish()
        else:
            if posts == 2:
                c.publish()
                self.exit('success')
            else:
                c.publish()
                self.exit('fail')


class FindGate(smach.StateMachine):

    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'])
        with self:
            smach.StateMachine.add('LOCATE_POLE', basic_states.LocateObject('start_gate_post', yaw_speed=-20), transitions={'success': 'YAW_TO_GATE','fail': 'fail',
               'timeout': 'fail'
               })
            smach.StateMachine.add('YAW_TO_GATE', YawToGate(), transitions={'success': 'success','timeout': 'fail',
               'fail': 'fail'
               })


class CenterGate(SubscribeState):

    def __init__(self, distance, max_duration=45, gate_width=3):
        SubscribeState.__init__(self, 'vision/relative', ObstaclePosArray, self.obstacle_callback, input_keys=[
         'side'], outcomes=[
         'success', 'fail'], timeout=max_duration)
        self.max_horizontal_error = 0.2
        self.max_distance_error = 0.4
        self.gate_width = gate_width
        self.distance_goal = distance

    def obstacle_callback(self, obstacle_msg, user_data):
        gate_posts = [ obstacle for obstacle in obstacle_msg.data if obstacle.name == 'start_gate_post'
                     ]
        if len(gate_posts) != 2:
            self.exit('fail')
            return
        y_average = (gate_posts[0].y + gate_posts[1].y) / 2
        if user_data.side == 'left' or user_data.side == 'black':
            target = -1 * self.gate_width / 4.0
        else:
            target = self.gate_width / 4.0
        y_error = y_average - target
        x_average = (gate_posts[0].x + gate_posts[1].x) / 2
        x_error = x_average - self.distance_goal
        y_good = abs(y_error) <= self.max_horizontal_error
        x_good = abs(x_error) <= self.max_distance_error
        c = control_wrapper.control_wrapper()
        c.levelOut()
        c.forwardError(0)
        c.strafeLeftError(0)
        if not y_good:
            rospy.loginfo(('Strafe error: {}').format(y_error))
            c.strafeLeftError(y_error)
        if not x_good:
            rospy.loginfo(('Forward error: {}').format(x_error))
            c.forwardError(x_error)
        c.publish()
        if y_good and x_good:
            self.exit('success')


class GetGateAngle(SubscribeState):

    def __init__(self, max_duration=45, gate_width=3):
        SubscribeState.__init__(self, 'vision/relative', ObstaclePosArray, self.obstacle_callback, outcomes=[
         'success', 'fail'], output_keys=[
         'yaw'], timeout=max_duration)
        self.max_horizontal_error = 0.2
        self.max_distance_error = 0.4

    def obstacle_callback(self, obstacle_msg, user_data):
        gate_posts = [ obstacle for obstacle in obstacle_msg.data if obstacle.name == 'start_gate_post'
                     ]
        if len(gate_posts) != 2:
            self.exit('fail')
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

    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=[
         'success', 'fail'], input_keys=[
         'target'])
        with self:
            smach.StateMachine.add('CALCULATE_MOVEMENT', GetGateAngle(), transitions={'success': 'YAW_TOWARDS_GATE','timeout': 'fail',
               'fail': 'fail'
               }, remapping={'yaw': 'angle'})
            smach.StateMachine.add('YAW_TOWARDS_GATE', basic_states.YawRelative(), transitions={'success': 'CENTER_GATE','timeout': 'fail'
               }, remapping={'yaw_left': 'angle'})
            smach.StateMachine.add('CENTER_GATE', CenterGate(2.0), transitions={'success': 'success','timeout': 'fail',
               'fail': 'fail'
               }, remapping={'side': 'target'})


class GateTask(smach.StateMachine):

    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'], input_keys=['target_side'])
        with self:
            smach.StateMachine.add('INITIAL_DEPTH', basic_states.GoToDepth(1.0), transitions={'success': 'FIND_GATE','timeout': 'fail',
               'fail': 'fail'
               })
            smach.StateMachine.add('FIND_GATE', FindGate(), transitions={'success': 'TARGET_GATE','fail': 'fail'
               })
            smach.StateMachine.add('TARGET_GATE', TargetGate(), transitions={'success': 'BLIND_FORWARD','fail': 'BLIND_FORWARD'
               }, remapping={'target': 'target_side'})
            smach.StateMachine.add('BLIND_FORWARD', basic_states.BlindRam(8.0, ramming_speed=1.0), transitions={'success': 'success'})


if __name__ == '__main__':
    rospy.init_node('ai')
    while rospy.get_time() == 0:
        continue

    sm = smach.StateMachine(outcomes=['success', 'fail'])
    sm.userdata.side = 'right'
    with sm:
        smach.StateMachine.add('CASINO_GATE', GateTask(), transitions={'success': 'success','fail': 'fail'
           }, remapping={'target_side': 'side'})
    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()
# okay decompiling smach_casino_gate.pyc
