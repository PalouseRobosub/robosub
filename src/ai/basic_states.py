"""
Author: Ryan Summers
Date: 1-2-2018

Description: Describes a number of basic Smach states for AI development.
"""
import control_wrapper
import numpy as np
import rospy
import smach
import tf
import util

from geometry_msgs.msg import QuaternionStamped
from robosub_msgs.msg import Float32Stamped, DetectionArray, Detection
from std_srvs.srv import Empty
from SubscribeState import SubscribeState


class Stabilize(SubscribeState):
    """Waits a specified timeout to check for stable tilt.

    attributes:
        stable_duration: The length (in seconds) that the sub must remain stable
            for.
        max_tilt: The maximum angle (in degrees) that the sub may pitch or roll
            to during a stable period.
    """

    def __init__(self, max_duration=15, max_tilt=5, stable_duration=2):
        """Initialization.

        Args:
            max_duration: The maximum duration of the state in seconds.
            max_tilt: The maximum roll or pitch allowed in degrees.
            stable_duration: The duration of time that stability must be
                maintained in seconds.
            """
        SubscribeState.__init__(self,
                                'orientation',
                                QuaternionStamped,
                                self.orientation_callback,
                                outcomes=['success', 'fail'],
                                setup_callback=self.setup,
                                timeout=max_duration)
        self.stable_duration = 2
        self.max_tilt = 5


    def setup(self):
        """Initializes state timeouts."""
        self.stable_timeout = rospy.get_time() + self.stable_duration


    def orientation_callback(self, orientation_msg, user_data):
        """Callback message for orientation messages."""
        q = orientation_msg.quaternion
        euler = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
        pitch = euler[1] * 180 / np.pi
        roll = euler[0] * 180 / np.pi

        if abs(roll) > self.max_tilt or abs(pitch) > self.max_tilt:
            self.stable_timeout = rospy.get_time() + self.stable_duration
        else:
            if rospy.get_time() > self.stable_timeout:
                self.exit('success')

        c = control_wrapper.control_wrapper()
        c.levelOut()
        c.strafeLeftError(0)
        c.forwardError(0)
        c.publish()


class BlindStrafe(smach.State):
    """ Blindly strafes for a period of time.

    Attributes:
        speed: The strafing speed (strafe error).
        duration: The duration for the blind strafe in seconds.
    """

    def __init__(self, duration, strafe_speed=1):
        """Initializes the SMACH state.

        Args:
            duration: The duration of the ram in seconds.
            strafe_speed: The strafing speed (strafe error) to use.
        """
        smach.State.__init__(self,
                             outcomes=['success'],
                             input_keys=[],
                             output_keys=[])
        self.speed = strafe_speed
        self.duration = duration


    def execute(self, user_data):
        """Executes the SMACH state."""
        c = control_wrapper.control_wrapper()
        c.levelOut()
        c.strafeLeftError(self.speed)

        end_time = rospy.get_time() + self.duration
        r = rospy.Rate(5)
        while rospy.get_time() < end_time:
            c.publish()
            r.sleep()
            continue

        c.forwardError(0.0)

        c.publish()
        r.sleep()
        c.publish()

        return 'success'


class BlindRam(smach.State):
    """ Blindly rams forward for a period of time.

    Attributes:
        speed: The ramming speed (forward error).
        duration: The duration for the blind ram in seconds.
    """

    def __init__(self, duration, ramming_speed=1):
        """Initializes the SMACH state.

        Args:
            duration: The duration of the ram in seconds.
            ramming_speed: The ramming speed (forward error) to use.
        """
        smach.State.__init__(self,
                             outcomes=['success'],
                             input_keys=[],
                             output_keys=[])
        self.speed = ramming_speed
        self.duration = duration


    def execute(self, user_data):
        """Executes the SMACH state."""
        c = control_wrapper.control_wrapper()
        c.levelOut()
        c.forwardError(self.speed)

        end_time = rospy.get_time() + self.duration
        r = rospy.Rate(5)
        while rospy.get_time() < end_time:
            c.publish()
            r.sleep()
            continue

        c.forwardError(0.0)

        c.publish()
        r.sleep()
        c.publish()

        return 'success'


class YawRelative(SubscribeState):
    """Yaws relative to the starting yaw and waits for stability.

    Attributes:
        target_yaw: The target yaw of the state.
        max_error: The maximum allowed error in yaw for success (in degrees).

    """
    def __init__(self, max_duration=30, max_error=5):
        """Initializes the SMACH state.

        Args:
            max_duration: The maximum duration of the state in seconds.
            max_error: The maximum yaw error allowed in degrees.
        """
        SubscribeState.__init__(self,
                                'orientation',
                                QuaternionStamped,
                                self.orientation_callback,
                                outcomes=['success'],
                                input_keys=['yaw_left'],
                                timeout=max_duration,
                                setup_callback=self.setup)
        self.target_yaw = None
        self.max_error = max_error


    def setup(self):
        """Setup callback function for when the state begins execution."""
        self.target_yaw = None


    def orientation_callback(self, orientation_msg, user_data):
        q = orientation_msg.quaternion
        euler = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
        yaw = euler[2] * 180 / np.pi
        rospy.logdebug('Yaw is {} -> Target: {}'.format(yaw, self.target_yaw))

        if self.target_yaw is None:
            self.target_yaw = util.wrap_yaw(yaw + user_data.yaw_left)
            c = control_wrapper.control_wrapper()
            c.levelOut()
            c.yawLeftRelative(user_data.yaw_left)
            c.forwardError(0)
            c.strafeLeftError(0)
            c.publish()
            return

        c = control_wrapper.control_wrapper()
        c.publish()
        if abs(util.wrap_yaw(self.target_yaw - yaw)) < self.max_error:
            self.exit('success')


class GoToDepth(SubscribeState):
    """State the goes to a specific depth.

    attributes:
        depth: The target depth (in meters) to dive to.
        max_error: The maximum error in depth that is acceptable (positive or
            negative).
    """

    def __init__(self, depth, max_duration=10, max_error=0.2):
        """Initializes the state.

        Args:
            depth: The absolute depth (below water) to dive to.
            max_duration: The max duration (in seconds) that the state may run
                for.
            max_error: The maximum depth error (in meters) that denotes success.
        """
        SubscribeState.__init__(self,
                                'depth',
                                Float32Stamped,
                                self.depth_callback,
                                outcomes=['success', 'fail'],
                                timeout=max_duration)
        self.depth = -1 * abs(depth)
        self.max_error = max_error


    def depth_callback(self, depth_msg, user_data):
        """Callback for ROS depth messages."""
        c = control_wrapper.control_wrapper()
        c.diveAbsolute(self.depth)
        c.levelOut()
        c.publish()

        if abs(abs(depth_msg.data) - abs(self.depth)) < self.max_error:
            self.exit('success')


class LocateObject(SubscribeState):
    """Rotates the sub in a circle until an object is detected.

    Attributes:
        label: The label of the object that is being searched for.
        yaw_speed: The speed at which the submarine yaws to find objects.
        fov_scale: The angle (in degrees) that the front camera can see.
    """

    def __init__(self,
                 label,
                 yaw_speed=20,
                 fov_scale=90,
                 max_duration=30):
        """Initializes the state.

        Args:
            label: The label of the object that is being searched for.
            yaw_speed: The speed at which the submarine yaws to find objects.
            fov_scale: The field of view (in degrees) of the front camera.
            max_duration: The maximum length of the state in seconds.
        """
        SubscribeState.__init__(self,
                                'vision/left',
                                DetectionArray,
                                self.detection_callback,
                                outcomes=['success', 'fail'],
                                timeout=max_duration)
        self.label = label
        self.yaw_speed = yaw_speed
        self.fov_scale = fov_scale


    def detection_callback(self, detections, user_data):
        """Callback for detected objects."""
        relevent_detections = util.filterByLabel(detections.detections,
                self.label)
        detection = util.getMostProbable(relevent_detections)

        c = control_wrapper.control_wrapper()
        c.levelOut()
        if detection is None:
            rospy.logdebug('{} was not found. Searching left {} '
                    'degrees.'.format(self.label, self.yaw_speed))

            c.yawLeftRelative(self.yaw_speed)
            c.publish()
        else:
            rospy.logdebug('{} was found.'.format(self.label))
            if 0.25 <= detection.x <= 0.75:
                c.yawLeftRelative(0)
                c.publish()
                self.exit('success')
            else:
                rospy.logdebug('Centering camera on {}'.format(self.label))
                c.yawLeftRelative((0.5 - detection.x) / 0.5 *
                        (self.fov_scale / 2))
                c.publish()


class FireTorpedo(smach.State):
    """Fires a single torpedo."""

    def __init__(self):
        """Initializes the state."""
        smach.State.__init__(self,
                             outcomes=['success'],
                             input_keys=[],
                             output_keys=[])


    def execute(self, user_data):
        """Executes the state."""
        rospy.logdebug('Waiting for shoot_torpedo server.')
        rospy.wait_for_service('shoot_torpedo')
        service = rospy.ServiceProxy('shoot_torpedo', Empty)

        rospy.loginfo('Firing torpedo.')
        service()
        return 'success'


class DropMarker(smach.State):
    """Drops a single marker."""

    def __init__(self):
        """Initializes the state."""
        smach.State.__init__(self,
                             outcomes=['success'],
                             input_keys=[],
                             output_keys=[])


    def execute(self, user_data):
        """Executes the state."""
        rospy.logdebug('Waiting for drop_marker server.')
        rospy.wait_for_service('drop_marker')
        service = rospy.ServiceProxy('drop_marker', Empty)

        rospy.loginfo('Dropping marker.')
        service()
        return 'success'
