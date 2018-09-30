#!/usr/bin/python2
"""
Description: A ROS SMACH implementation for the surface in square task.
"""

from operator import add, sub
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64
from robosub_msgs.msg import DetectionArray, Detection, Float32Stamped
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
from SubscribeState import SubscribeState, MultiSubscribeState
import basic_states
import util
import control_wrapper
import enum


class MoveToPinger(MultiSubscribeState):
    """Moves the submarine to the pinger.

    Attributes:
        forward_speed: The speed at which to move towards the pinger.
        downward_angle_success: The angle of the downward cone that the pinger
            must lie in to denote success.
    """

    def __init__(self, max_duration=9999, forward_speed=4, successful_angle=15):
        """Initializes the state.

        Args:
            max_duration: The maximum length of the state in seconds.
            forward_speed: The speed at which to move forward.
            successful_angle: The downward angle in degrees that the pinger
                must lie in for success.
        """
        topics = ['hydrophones/accurate_bearing', 'hydrophones/bearing/magnitude']
        msg_types = [Vector3Stamped, Float32Stamped]
        callbacks = [self.pinger_callback, self.magnitude_callback]

        MultiSubscribeState.__init__(self,
                                topics,
                                msg_types,
                                callbacks,
                                outcomes=['success', 'fail'],
                                timeout=max_duration)
        
        print ("init find pinger")

        # parameters
        self.forward_speed = forward_speed
        self.downward_angle_success = successful_angle
        self.desired_success_count = 1
        self.max_pings_after_threshold = 5
        self.theta_z_threshold = 50

        # variables
        self.success_count = 0
        self.pings_after_threshold = 0
        self.theta_z_threshold_met = False
        self.last_theta_z = 99

    def magnitude_callback(self, magnitude, user_data):
        if self.last_theta_z < self.theta_z_threshold:
            self.theta_z_threshold_met = True

        if self.theta_z_threshold_met == True:
            self.pings_after_threshold += 1
            print ("pings after threshold: {}".format(self.pings_after_threshold))

        if self.pings_after_threshold >= self.max_pings_after_threshold:
            self.exit('success')
            return

        print ("magnitude: {}".format(magnitude.data))
        magnitude = magnitude.data
        # if magnitude is bad either search or continue
        if magnitude < 0.8 or magnitude > 1.2:
            c = control_wrapper.control_wrapper()
            c.levelOut()
            c.strafeLeftError(0.0)

            # if last z value was close stop and spin
            if self.last_theta_z < 50:
                print ("bad ping rotating")
                c.yawLeftRelative(45)
            else:
                # otherwise keep going the same way
                print ("bad ping continuing forward")
                c.forwardError(self.forward_speed / 2)

            c.publish()
            return

    def pinger_callback(self, bearing_stamped, user_data):
        """ROS callback for pinger bearing message."""


        print("pinger callback")
        bearing = bearing_stamped.vector
        relative_yaw = 180 / np.pi * np.arctan2(bearing.y, bearing.x)

        c = control_wrapper.control_wrapper()
        c.levelOut()
        c.strafeLeftError(0.0)
        c.yawLeftRelative(relative_yaw)

        rospy.logdebug('Turning {} degrees left to track pinger'.format(
                    relative_yaw))
        print('Turning {} degrees left to track pinger'.format(relative_yaw))

        # Calculate the downward angle to the pinger.
        theta_z = 180 / np.pi * np.arctan2(np.sqrt(bearing.x**2 + bearing.y**2),
                                           abs(bearing.z))
        self.last_theta_z = theta_z

        rospy.logdebug('Pinger downward angle is {} degrees'.format(theta_z))
        print('Pinger downward angle is {} degrees'.format(theta_z))

        # Check the current downward angle against the successful angle
        # requirement.
        if theta_z < self.downward_angle_success:
            c.forwardError(0)
            c.publish()
            self.success_count += 1

            print('Success! count={}'.format(self.success_count))

            if self.success_count >= self.desired_success_count:
                self.exit('success')
        else:
            # Ensure the sub is pointing towards the pinger before moving
            # forward.
            self.success_count = 0

            # adjust forward speed based on how close we are to the pinger
            forward_speed = self.forward_speed
            if theta_z < 50:
                forward_speed /= 4.0

            if abs(relative_yaw) < 10:
                c.forwardError(forward_speed)
            else:
                c.forwardError(forward_speed / 2)
            c.publish()



class RouletteTask(smach.StateMachine):
    """Main roulette state task."""

    def __init__(self):
        """Initializes the roulette task."""
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'])

        with self:
            # First, dive to a certain depth and search for the pinger.
            """
            smach.StateMachine.add('DIVE_PINGER', basic_states.GoToDepth(0.5),
                    transitions={'success': 'FIND_PINGER',
                                 'fail': 'fail',
                                 'timeout': 'fail'})
            """

            smach.StateMachine.add('FIND_PINGER', MoveToPinger(),
                    transitions={'success': 'SURFACE',
                                 'fail': 'fail',
                                 'timeout': 'fail'})

            smach.StateMachine.add('SURFACE', basic_states.GoToDepth(0.0),
                    transitions={'success': 'success',
                                 'timeout': 'fail'})



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
