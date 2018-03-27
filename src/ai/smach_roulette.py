#!/usr/bin/python
"""
Author: Ryan Summers
Date: 1-2-2018

Description: A ROS SMACH implementation for the roulette wheel task.
"""

from operator import add, sub
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3Stamped
from robosub_msgs.msg import DetectionArray, Detection
from start_switch import start_switch
import cv2
import cv_bridge
import math
import numpy as np
import rospy
import smach
import smach_ros
import tf

from RouletteWheel import *
from SubscribeState import SubscribeState, SynchronousSubscribeState
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
    """

    def __init__(self, max_duration=120, forward_speed=3, successful_angle=15):
        """Initializes the state.

        Args:
            max_duration: The maximum length of the state in seconds.
            forward_speed: The speed at which to move forward.
            successful_angle: The downward angle in degrees that the pinger
                must lie in for success.
        """
        SubscribeState.__init__(self,
                                'hydrophones/bearing',
                                Vector3Stamped,
                                self.pinger_callback,
                                outcomes=['success', 'fail'],
                                timeout=max_duration)
        self.forward_speed = forward_speed
        self.downward_angle_success = successful_angle


    def pinger_callback(self, bearing_stamped, user_data):
        """ROS callback for pinger bearing message."""
        bearing = bearing_stamped.vector
        relative_yaw = 180 / np.pi * np.arctan2(bearing.y, bearing.x)

        c = control_wrapper.control_wrapper()
        c.levelOut()
        c.strafeLeftError(0.0)
        c.yawLeftRelative(relative_yaw)

        rospy.logdebug('Turning {} degrees left to track pinger'.format(
                    relative_yaw))

        # Calculate the downward angle to the pinger.
        theta_z = 180 / np.pi * np.arctan2(np.sqrt(bearing.x**2 + bearing.y**2),
                                           abs(bearing.z))

        rospy.logdebug('Pinger downward angle is {} degrees'.format(theta_z))

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


class CenterDownwardCamera(SubscribeState):
    """Centers the downward camera on the roulette wheel.

    Attributes:
        speed: The speed factor to use for movement.
        center_percentage: The percentage of the image that the center of the
            roulette wheel must lie in.
        retry_count: The maximum number of times that a detection can arrive
            without the roulette wheel marked.
        tries: The number of times a detection has arrived without the roulette
            wheel marked.
    """

    def __init__(self,
                 max_retry_count=5,
                 center_percentage=20,
                 max_duration=50):
        """Initializes the state.

        Args:
            max_retry_count: The maximum number of times that a detection can
                arrive without the roulette wheel marked.
            center_percentage: The percentage of the image that the center of
                the roulette wheel must lie on.
            max_duration: The max duration of the state in seconds.
        """
        SubscribeState.__init__(self,
                                'vision/bottom',
                                DetectionArray,
                                self.camera_callback,
                                outcomes=['success', 'fail'],
                                timeout=max_duration)
        self.speed = 1
        self.center_percentage = center_percentage
        self.tries = 0
        self.retry_count = max_retry_count


    def camera_callback(self, detection_msg, user_data):
        """ROS callback for bottom camera detection messages."""
        wheels = [x for x in detection_msg.detections
                  if x.label == 'roulette_wheel']
        detection = util.getMostProbable(detection_msg.detections, thresh=0.5)

        # If the roulette wheel is available.
        if detection is None:
            if self.tries >= self.retry_count:
                self.exit('fail')
            self.tries = self.tries + 1
            return

        x_error = (detection.x - 0.5) / 0.5
        y_error = (detection.y - 0.5) / 0.5
        c = control_wrapper.control_wrapper()
        c.levelOut()

        # Check to see if the image is centered for the X and Y axes.
        x_good = abs(x_error) * 100 < self.center_percentage / 2
        y_good = abs(y_error) * 100 < self.center_percentage / 2

        if not x_good:
            c.strafeLeftError(-1 * x_error * self.speed)
            rospy.logdebug('Centering X by strafing left {}'.format(-1 *
                        self.speed * x_error))
        else:
            c.strafeLeftError(0)
            rospy.logdebug('X centered')

        if not y_good:
            c.forwardError(-1 * y_error * self.speed)
            rospy.logdebug('Centering Y by going forward {}'.format(-1 *
                        self.speed * y_error))
        else:
            c.forwardError(0)
            rospy.logdebug('Y centered')

        c.publish()

        if x_good and y_good:
            self.exit('success')


class CenterAbove(smach.StateMachine):
    """Centers the submarine above the roulette wheel."""

    def __init__(self):
        """Initializes the state."""
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'])

        with self:
            # First, dive down to see the wheel.
            smach.StateMachine.add('DIVE_ROULETTE', basic_states.GoToDepth(2.5),
                    transitions={'success': 'STABILIZE',
                                 'fail': 'fail',
                                 'timeout': 'STABILIZE'})

            # Next, ensure there is not too much tilting.
            smach.StateMachine.add('STABILIZE', basic_states.Stabilize(),
                    transitions={'success': 'CENTER',
                                 'fail': 'fail',
                                 'timeout': 'fail'})

            # Finally, center the submarine above the roulette wheel.
            smach.StateMachine.add('CENTER', CenterDownwardCamera(),
                    transitions={'success': 'success',
                                 'fail': 'fail',
                                 'timeout': 'fail'})


class TargetColor(SynchronousSubscribeState):
    """Centers above the nearest colored slice of the roulette wheel.

    Attributes:
        color: The RouletteWheel.Color type to center above.
        speed: The speed factor for translating the submarine to the color
            slice.
        center_percentage: The percentage of the image that the slice must
            reside in for success.
        retry_count: The maximum number of images that can arrive without
            the roulette wheel in the image.
        bridge: A CvBridge object for converting ROS Image messages to OpenCV.
    """

    def __init__(self, color, speed=1.3, max_retries=5, max_duration=45):
        """Initializes the state.

        Args:
            color: The RouletteWheel.Color type to search for.
            speed: The speed of movement.
            max_retries: The maximum number of images that can arrive without
                a roulette wheel in the image.
            max_duration: The max duration of the state in seconds.
        """
        SynchronousSubscribeState.__init__(self,
                                'camera/bottom/undistorted',
                                Image,
                                'vision/bottom',
                                DetectionArray,
                                1.0,
                                self.camera_callback,
                                outcomes=['success', 'fail'],
                                timeout=max_duration)
        self.color = color
        self.speed = speed
        self.center_percentage = 5
        self.bridge = cv_bridge.CvBridge()
        self.tries = 0
        self.retry_count = max_retries


    def camera_callback(self, image_msg, detection_msg, user_data):
        img = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        img = cv2.resize(img, (0, 0), fx=0.3, fy=0.3)

        height = img.shape[0]
        width = img.shape[1]

        wheels = [x for x in detection_msg.detections
                  if x.label == 'roulette_wheel']

        wheel_detection = util.getMostProbable(wheels, thresh=0.5)

        if wheel_detection is None:
            if self.tries > self.retry_count:
                self.exit('fail')
            self.tries = self.tries + 1
            return

        origin = (wheel_detection.x * width, wheel_detection.y * height)
        length = wheel_detection.width * width
        height = wheel_detection.height * height

        upper_left = (int(origin[0] - length / 2), int(origin[1] - height / 2))
        lower_right = (int(origin[0] + length / 2), int(origin[1] + height / 2))

        # Mask away everything except the detection box.
        mask = np.zeros(shape=img.shape, dtype=np.uint8)
        cv2.rectangle(mask, upper_left, lower_right, (255, 255, 255), -1)
        img = cv2.bitwise_and(img, mask)

        wheel = RouletteWheel(img)

        if len(wheel.slices) == 0:
            self.exit('fail')
            return

        # Filter the slices to the color of interest.
        color_slices = []
        for color_slice in wheel.slices:
            if color_slice.color == self.color:
                color_slices.append(color_slice)

        if len(color_slices) != 2:
            rospy.loginfo('Did not find two slices with color '
                    '{}'.format(self.color))
            self.exit('fail')
            return

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

        x_error = float(detection.x - 0.5) / 0.5
        y_error = float(detection.y - 0.5) / 0.5
        x_good = 100 * abs(x_error) < self.center_percentage / 2
        y_good = 100 * abs(y_error) < self.center_percentage / 2

        c = control_wrapper.control_wrapper()
        c.levelOut()

        if not x_good:
            c.strafeLeftError(-1 * x_error * self.speed)
            rospy.logdebug('Centering X by strafing left {}'.format(-1 *
                        self.speed * x_error))
        else:
            c.strafeLeftError(0)
            rospy.logdebug('X centered')

        if not y_good:
            c.forwardError(-1 * y_error * self.speed)
            rospy.logdebug('Centering Y by going forward {}'.format(-1 *
                        self.speed * y_error))
        else:
            c.forwardError(0)
            rospy.logdebug('Y centered')

        c.publish()

        if x_good and y_good:
            self.exit('success')


class RouletteTask(smach.StateMachine):
    """Main roulette state task."""

    def __init__(self):
        """Initializes the roulette task."""
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'])

        with self:
            # First, dive to a certain depth and search for the pinger.
            smach.StateMachine.add('DIVE_PINGER', basic_states.GoToDepth(1.0),
                    transitions={'success': 'FIND_PINGER',
                                 'fail': 'fail',
                                 'timeout': 'fail'})

            smach.StateMachine.add('FIND_PINGER', MoveToPinger(),
                    transitions={'success': 'STABILIZE',
                                 'fail': 'fail',
                                 'timeout': 'fail'})

            smach.StateMachine.add('STABILIZE', basic_states.Stabilize(),
                    transitions={'success': 'LOCATE_ROULETTE',
                                 'fail': 'fail',
                                 'timeout': 'LOCATE_ROULETTE'})

            # Next, locate the roulette wheel and center above it.
            smach.StateMachine.add('LOCATE_ROULETTE', CenterAbove(),
                    transitions={'success': 'DIVE_TARGET',
                                 'fail': 'fail'})

            # Now, dive lower and center above the desired color.
            smach.StateMachine.add('DIVE_TARGET', basic_states.GoToDepth(3),
                    transitions={'success': 'CENTER_TARGET',
                                 'fail': 'fail',
                                 'timeout': 'fail'})

            smach.StateMachine.add('CENTER_TARGET', TargetColor(Color.GREEN),
                    transitions={'success': 'DROP_TARGET',
                                 'fail': 'fail',
                                 'timeout': 'DROP_TARGET'})

            # Finally, drop the marker.
            smach.StateMachine.add('DROP_TARGET',
                    basic_states.DropMarker(),
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
