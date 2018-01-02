#!/usr/bin/python
from operator import add, sub
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3Stamped
from rs_yolo.msg import DetectionArray, Detection
import cv2
import cv_bridge
import math
import numpy as np
import rospy
import smach
import smach_ros
import tf

from RouletteWheel import *
from SubscribeState import SubscribeState
import basic_states
import util
import control_wrapper
import enum


class MoveToPinger(SubscribeState):
    """Moves the submarine to the pinger.

    Attributes:
        forward_speed: The speed at which to move towards the pinger.
        downward_angle_success: The angle of the downward cone that the pinger
            must lie in to denote success.
        search_duration: The maximum length (in seconds) that the state may run
            for.
        timeout: The ROS time at which the task will exit.
    """

    def __init__(self, max_duration=120, forward_speed=3, successful_angle=15):
        """Initializes the state.

        Args:
            max_duration: The maximum length of the state in seconds.
            forward_speed: The speed at which to move forward.
            successful_angle: The downward angle that the pinger must lie in for
                success.
        """
        SubscribeState.__init__(self,
                                'hydrophones/bearing',
                                Vector3Stamped,
                                self.pinger_callback,
                                outcomes=['success', 'fail'],
                                setup_callback=self.setup)
        self.forward_speed = forward_speed
        self.downward_angle_success = successful_angle
        self.search_duration = max_duration
        self.timeout = 0


    def setup(self):
        """Sets up the state timeout."""
        self.timeout = rospy.get_time() + self.search_duration


    def pinger_callback(self, bearing_stamped, user_data):
        """ROS callback for pinger bearing message."""
        bearing = bearing_stamped.vector
        relative_yaw = 180 / np.pi * np.arctan2(bearing.y, bearing.x)

        c = control_wrapper.control_wrapper()
        c.levelOut()
        c.strafeLeftError(0.0)
        c.yawLeftRelative(relative_yaw)

        rospy.loginfo( \
                'Turning {} degrees left to track pinger'.format(relative_yaw))

        # Calculate the downward angle to the pinger.
        theta_z = 180 / np.pi * np.arctan2(np.sqrt(bearing.x**2 + bearing.y**2),
                                           abs(bearing.z))

        rospy.loginfo('Pinger downward angle is {} degrees'.format(theta_z))

        # Check the current downward angle against the successful angle
        # requirement.
        if theta_z < self.downward_angle_success:
            c.forwardError(0)
            c.publish()
            self.exit('success')
        else:
            # Ensure the sub is pointing towards the pinger before moving
            # forward.
            if abs(relative_yaw) < 10:
                c.forwardError(self.forward_speed)
            else:
                c.forwardError(0)
            c.publish()

        # Check the timeout of the state.
        if rospy.get_time() > self.timeout:
            rospy.loginfo('Failing due to state timeout')
            self.exit('fail')


class center_downward(SubscribeState):
    def __init__(self, label):
        SubscribeState.__init__(self, 'camera/bottom/undistorted', Image, self.camera_callback, outcomes=['success', 'fail'], setup_callback=self.setup)
        self.label = label
        self.speed = 1
        self.bridge = cv_bridge.CvBridge()
        self.center_percentage = 20


    def setup(self):
        self.timeout = rospy.get_time() + 5


    def camera_callback(self, image_msg, user_data):
        img = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        img = cv2.resize(img, (0, 0), fx=0.3, fy=0.3)

        wheel = RouletteWheel(img)
        if len(wheel.slices) == 0:
            self.exit('fail')

#        cv2.imshow('Target', wheel.image)
#        cv2.waitKey(1)

        detection = Detection()
        detection.x = float(wheel.origin[0]) / img.shape[1]
        detection.y = float(wheel.origin[1]) / img.shape[0]

        if detection is None:
            if self.timeout < rospy.get_time():
                self.exit('fail')
        else:
            x_good = False
            y_good = False
            x_error = (detection.x - 0.5) / 0.5
            y_error = (detection.y - 0.5) / 0.5
            c = control_wrapper.control_wrapper()
            c.levelOut()
            if 100 * abs(x_error) > self.center_percentage / 2:
                c.strafeLeftError(-1 * x_error * self.speed)
                rospy.loginfo('Centering X by strafing left {}'.format(-1 * self.speed * x_error))
            else:
                c.strafeLeftError(0)
                x_good = True
                rospy.loginfo('X centered')

            if 100 * abs(y_error) > self.center_percentage / 2:
                c.forwardError(-1 * y_error * self.speed)
                rospy.loginfo('Centering Y by going forward {}'.format(-1 *self.speed * y_error))
            else:
                c.strafeLeftError(0)
                y_good = True
                rospy.loginfo('Y centered')

            c.publish()

            if x_good and y_good:
                self.exit('success')


class center_above(smach.StateMachine):
    def __init__(self, label):
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'])

        with self:
#            smach.StateMachine.add('LOCATE_OBJECT', locate_object(label), \
#                    transitions={'success': 'CENTER', \
#                                 'fail': 'fail'})

#            smach.StateMachine.add('MOVE_TO', move_to(label), \
#                    transitions={'success': 'CENTER', \
#                                 'fail': 'CENTER'})

            smach.StateMachine.add('DIVE_ROULETTE', basic_states.GoToDepth(2.5),
                    transitions={'success': 'STABILIZE', 'fail': 'fail'})

            smach.StateMachine.add('STABILIZE', basic_states.Stabilize(),
                    transitions={'success': 'CENTER', 'fail': 'fail'})

            smach.StateMachine.add('CENTER', center_downward(label), \
                    transitions={'success': 'success', \
                                 'fail': 'fail'})


class target_color(SubscribeState):
    def __init__(self, color):
        SubscribeState.__init__(self, 'camera/bottom/undistorted', Image, self.camera_callback, outcomes=['success', 'fail'])
        self.color = color
        self.speed = 1
        self.center_percentage = 5
        self.bridge = cv_bridge.CvBridge()


    def camera_callback(self, image_msg, user_data):
        img = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        img = cv2.resize(img, (0, 0), fx=0.3, fy=0.3)

        wheel = RouletteWheel(img)
        if len(wheel.slices) == 0:
            self.exit('fail')

        cv2.imshow('Target', wheel.image)
        cv2.waitKey(1)

        color_slices = []
        for color_slice in wheel.slices:
            if color_slice.color == self.color:
                color_slices.append(color_slice)

        if len(color_slices) != 2:
            rospy.loginfo('Did not find two slices with color {}'.format(self.color))
            self.exit('fail')

        # Find the slice that is closest to the center.
        distances = []
        for slc in color_slices:
            x = slc.origin[0] - img.shape[1] / 2
            y = slc.origin[1] - img.shape[0] / 2
            distances.append(math.sqrt(x**2 + y**2))

        if distances[0] < distances[1]:
            left_slice_center = color_slices[0].origin
        else:
            left_slice_center = color_slices[1].origin

        detection = Detection()
        detection.x = float(left_slice_center[0]) / img.shape[1]
        detection.y = float(left_slice_center[1]) / img.shape[0]

        x_error = detection.x - 0.5
        y_error = detection.y - 0.5
        x_good = False
        y_good = False
        c = control_wrapper.control_wrapper()
        c.levelOut()

        if 100 * abs(x_error) > self.center_percentage / 2:
            c.strafeLeftError(-1 * x_error * self.speed)
            rospy.loginfo('Centering X by strafing left {}'.format(-1 * self.speed * x_error))
        else:
            c.strafeLeftError(0)
            x_good = True
            rospy.loginfo('X centered')

        if 100 * abs(y_error) > self.center_percentage / 2:
            c.forwardError(-1 * y_error * self.speed)
            rospy.loginfo('Centering Y by going forward {}'.format(-1 *self.speed * y_error))
        else:
            c.forwardError(0)
            y_good = True
            rospy.loginfo('Y centered')

        c.publish()
        if x_good and y_good:
            self.exit('success')


class RouletteTask(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'])

        with self:
            smach.StateMachine.add('DIVE_PINGER', basic_states.GoToDepth(1.5),
                    transitions={'success': 'FIND_PINGER', 'fail': 'fail'})

            smach.StateMachine.add('FIND_PINGER', MoveToPinger(),
                    transitions={'success': 'STABILIZE',
                                 'fail': 'fail'})

            smach.StateMachine.add('STABILIZE', basic_states.Stabilize(),
                    transitions={'success': 'LOCATE_ROULETTE', 'fail': 'fail'})

            smach.StateMachine.add('LOCATE_ROULETTE',
                    center_above('roulette_wheel'),
                    transitions={'success': 'DIVE_TARGET',
                                 'fail': 'fail'})

            smach.StateMachine.add('DIVE_TARGET', basic_states.GoToDepth(3),
                    transitions={'success': 'CENTER_TARGET', 'fail': 'fail'})

            smach.StateMachine.add('CENTER_TARGET', target_color(Color.GREEN),
                    transitions={'success': 'DROP_TARGET',
                                 'fail': 'fail'})

            smach.StateMachine.add('DROP_TARGET',
                    basic_states.DropMarker(outcomes=['success', 'fail']),
                    transitions={'success': 'success',
                                 'fail': 'fail'})

if __name__ == '__main__':
    rospy.init_node('ai')

    # Wait for ROS time to properly begin.
    while rospy.get_time() == 0:
        continue

    sm = smach.StateMachine(outcomes=['success', 'fail'])

    with sm:
        smach.StateMachine.add('ROULETTE', RouletteTask(),
                transitions={'success': 'success',
                             'fail': 'fail'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()
