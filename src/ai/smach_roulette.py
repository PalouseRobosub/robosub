#!/usr/bin/python
import rospy
import smach
import smach_ros
import tf
import util
import control_wrapper
from SubscribeState import SubscribeState
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped
from rs_yolo.msg import DetectionArray, Detection
import numpy as np

class move_to_pinger(SubscribeState):
    def __init__(self):
        SubscribeState.__init__(self, 'hydrophones/bearing', Vector3Stamped, self.pinger_callback, outcomes=['success', 'fail'])
        self.forward_speed = 3
        self.downward_angle_success = 15
        self.timeout = rospy.get_time() + 60


    def pinger_callback(self, bearing_stamped, user_data):
        bearing = bearing_stamped.vector
        c = control_wrapper.control_wrapper()
        c.levelOut()
        c.diveAbsolute(1.0)
        relative_yaw = 180 / np.pi * np.arctan2(bearing.y, bearing.x)
        rospy.loginfo('Turning {} degrees left to track pinger.'.format(relative_yaw))
        c.yawLeftRelative(relative_yaw)
        if rospy.get_time() > self.timeout:
            c.forwardError(0)
            c.publish()
            rospy.loginfo('Failing due to timeout. Now: {} Timeout: {}'.format(rospy.get_time(), self.timeout))
            self.exit('fail')

        theta_z = 180 / np.pi * np.arctan2(np.sqrt(bearing.x**2 + bearing.y**2), abs(bearing.z))
        rospy.loginfo('Pinger downward angle is {} degrees.'.format(theta_z))
        if theta_z < self.downward_angle_success:
            c.forwardError(0)
            c.diveAbsolute(2.0)
            c.publish()
            self.exit('success')
        else:
            c.forwardError(self.forward_speed)
            c.publish()


class locate_object(SubscribeState):
    def __init__(self, label):
        SubscribeState.__init__(self, 'vision', DetectionArray, self.detection_callback, outcomes=['success', 'fail'], setup_callback=self.setup)
        self.label = label
        self.depth = 1.0
        self.yaw_speed = 20
        self.fov_scale = 90


    def setup(self):
        self.timeout = rospy.get_time() + 30


    def detection_callback(self, detections, user_data):
        relevent_detections = util.filterByLabel(detections.detections, self.label, thresh=0.5)
        detection = util.getMostProbable(relevent_detections)

        c = control_wrapper.control_wrapper()
        c.levelOut()
        c.diveAbsolute(self.depth)
        if detection is None:
            rospy.loginfo('{} was not found. Searching left {} degrees.'.format(self.label, self.yaw_speed))
            if rospy.get_time() > self.timeout:
                self.exit('fail')

            c.yawLeftRelative(self.yaw_speed)
            c.publish()
        else:
            rospy.loginfo('{} was found.'.format(self.label))
            if 0.25 <= detection.x <= 0.75:
                c.yawLeftRelative(0)
                c.publish()
                self.exit('success')
            else:
                rospy.loginfo('Centering camera on {}'.format(self.label))
                c.yawLeftRelative((0.5 - detection.x) / 0.5 * (self.fov_scale / 2))
                c.publish()


class move_to(SubscribeState):
    def __init__(self, label):
        SubscribeState.__init__(self, 'vision', DetectionArray, self.detection_callback, outcomes=['success', 'fail'], setup_callback=self.setup)
        self.label = label
        self.forward_speed = 3
        self.was_tracking = False


    def setup(self):
        self.timeout = rospy.get_time() + 5


    def detection_callback(self, detections, user_data):
        relevent_detections = util.filterByLabel(detections.detections, self.label, thresh=0.5)
        detection = util.getMostProbable(relevent_detections)
        if detection is None:
            if rospy.get_time() > self.timeout:
                c = control_wrapper.control_wrapper()
                c.levelOut()
                c.setForward(0)
                c.publish()
                if self.was_tracking:
                    self.exit('success')
                else:
                    self.exit('fail')
        else:
            c = control_wrapper.control_wrapper()
            c.levelOut()
            c.yawLeftRelative((0.5 - detection.x) / 0.5 * (self.fov_scale / 2))

            if abs(detection.x - 0.5) < .2:
                c.setForward(self.forward_speed)
                self.was_tracking = True

            c.publish()

            self.timeout = rospy.get_time() + 5


class center_downward(SubscribeState):
    def __init__(self, label):
        SubscribeState.__init__(self, 'vision/down', DetectionArray, self.detection_callback, outcomes=['success', 'fail'], setup_callback=self.setup)
        self.label = label
        self.speed = 3


    def setup(self):
        self.timeout = rospy.get_time() + 5


    def detection_callback(self, detections, user_data):
        relevent_detections = util.filterByLabel(detections.detections, self.label, thresh=0.5)
        detection = util.getMostProbable(relevent_detections)

        if detection is None:
            if self.timeout < rospy.get_time():
                self.exit('fail')
        else:
            x_good = False
            y_good = False
            c = control_wrapper.control_wrapper()
            c.levelOut()
            if abs(detection.x - 0.5) > 0.1:
                c.setRelativeStrafe(-1 * (detection.x - 0.5) / 0.5 * self.speed)
            else:
                c.setRelativeStrafe(0)
                x_good = True
            if abs(detection.y - 0.5) > 0.1:
                c.setRelativeForward(-1 * (detection.y - 0.5) / 0.5 * self.speed)
            else:
                c.setRelativeStrafe(0)
                y_good = True
            c.publish()

            if x_good and y_good:
                self.exit('success')


class center_above(smach.StateMachine):
    def __init__(self, label):
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'])

        with self:
            smach.StateMachine.add('LOCATE_OBJECT', locate_object(label), \
                    transitions={'success': 'MOVE_TO', \
                                 'fail': 'fail'})

            smach.StateMachine.add('MOVE_TO', move_to(label), \
                    transitions={'success': 'CENTER', \
                                 'fail': 'CENTER'})

            smach.StateMachine.add('CENTER', center_downward(label), \
                    transitions={'success': 'success', \
                                 'fail': 'fail'})


class target_color(SubscribeState):
    def __init__(self, color):
        SubscribeState.__init__(self, 'camera/bottom/undistorted', Image, self.camera_callback, outcomes=['success', 'fail'])
        self.color = color


    def camera_callback(self, image, user_data):
        self.exit('success')


class drop_marker(smach.State):
    def __init__(self, color, outcomes, input_keys=[], output_keys=[]):
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)


    def execute(self, user_data):
        return 'success'


class roulette_task(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'])

        with self:
            smach.StateMachine.add('FIND_PINGER', move_to_pinger(),
                    transitions={'success': 'LOCATE_ROULETTE',
                                 'fail': 'fail'})

            smach.StateMachine.add('LOCATE_ROULETTE', center_above('roulette_wheel'),
                    transitions={'success': 'CENTER_TARGET',
                                 'fail': 'fail'})

            smach.StateMachine.add('CENTER_TARGET', target_color('red'),
                    transitions={'success': 'DROP_TARGET',
                                 'fail': 'fail'})

            smach.StateMachine.add('DROP_TARGET', drop_marker('red', outcomes=['success', 'fail']),
                    transitions={'success': 'success',
                                 'fail': 'fail'})

if __name__ == '__main__':

    rospy.init_node('ai')

    while rospy.get_time() == 0:
        continue

    sm = smach.StateMachine(outcomes=['success', 'fail'])

    with sm:
        smach.StateMachine.add('ROULETTE', roulette_task(),
                transitions={'success': 'success',
                             'fail': 'fail'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()
