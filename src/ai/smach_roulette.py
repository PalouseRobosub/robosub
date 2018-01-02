#!/usr/bin/python
import rospy
import smach
import cv2
import smach_ros
import tf
from operator import add, sub
from std_srvs.srv import Empty
import util
import control_wrapper
from SubscribeState import SubscribeState
from robosub.msg import Float32Stamped
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped
from rs_yolo.msg import DetectionArray, Detection
import numpy as np
import cv_bridge

class stabilize(SubscribeState):
    def __init__(self):
        SubscribeState.__init__(self, 'orientation', QuaternionStamped, self.orientation_callback, outcomes=['success', 'fail'], setup_callback=self.setup)
        self.timeout = 0
        self.stable_duration = 2
        self.max_tilt = 5


    def setup(self):
        self.timeout = rospy.get_time() + 15
        self.stable_timeout = rospy.get_time() + self.stable_duration


    def orientation_callback(self, orientation_msg, user_data):
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

        if rospy.get_time() > self.timeout:
            self.exit('fail')


class go_to_depth(SubscribeState):
    def __init__(self, depth):
        SubscribeState.__init__(self, 'depth', Float32Stamped, self.depth_callback, outcomes=['success', 'fail'], setup_callback=self.setup)
        self.timeout = 0
        self.depth = depth
        self.max_error = 0.2


    def setup(self):
        self.timeout = rospy.get_time() + 10


    def depth_callback(self, depth_msg, user_data):
        c = control_wrapper.control_wrapper()
        c.diveAbsolute(self.depth)
        c.levelOut()
        c.publish()

        if abs(abs(depth_msg.data) - abs(self.depth)) < self.max_error:
            self.exit('success')

        if rospy.get_time() > self.timeout:
            self.exit('fail')


class move_to_pinger(SubscribeState):
    def __init__(self):
        SubscribeState.__init__(self, 'hydrophones/bearing', Vector3Stamped, self.pinger_callback, outcomes=['success', 'fail'], setup_callback=self.setup)
        self.forward_speed = 3
        self.downward_angle_success = 15
        self.search_duration = 120
        self.timeout = 0


    def setup(self):
        self.timeout = rospy.get_time() + self.search_duration


    def pinger_callback(self, bearing_stamped, user_data):
        bearing = bearing_stamped.vector
        c = control_wrapper.control_wrapper()
        c.levelOut()
        c.strafeLeftError(0.0)
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
            c.publish()
            self.exit('success')
        else:
            if abs(relative_yaw) < 10:
                c.forwardError(self.forward_speed)
            else:
                c.forwardError(0)
            c.publish()


class locate_object(SubscribeState):
    def __init__(self, label):
        SubscribeState.__init__(self, 'vision', DetectionArray, self.detection_callback, outcomes=['success', 'fail'], setup_callback=self.setup)
        self.label = label
        self.yaw_speed = 20
        self.fov_scale = 90


    def setup(self):
        self.timeout = rospy.get_time() + 30


    def detection_callback(self, detections, user_data):
        relevent_detections = util.filterByLabel(detections.detections, self.label, thresh=0.5)
        detection = util.getMostProbable(relevent_detections)

        c = control_wrapper.control_wrapper()
        c.levelOut()
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

        hsv_im = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        red_low_hues = cv2.inRange(hsv_im, lower_red, upper_red)

        lower_red = np.array([160, 100, 100])
        upper_red = np.array([179, 255, 255])
        red_high_hues = cv2.inRange(hsv_im, lower_red, upper_red)
        red_filtered = cv2.addWeighted(red_high_hues, 1.0, red_low_hues, 1.0, 0)
        blurred = cv2.GaussianBlur(red_filtered, (9,9), 0)
        im, contours, hierarchy = cv2.findContours(blurred, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        sorted_contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
        points = np.concatenate(sorted_contours[:1])
        (x, y), radius = cv2.minEnclosingCircle(points)
        cv2.circle(img, (int(x), int(y)), int(radius), (0, 255, 0), 3)

#        cv2.imshow('Roulette wheel', img)
#        cv2.waitKey(1)
        detection = Detection()
        detection.x = x / img.shape[1]
        detection.y = y / img.shape[0]

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

            smach.StateMachine.add('DIVE_ROULETTE', go_to_depth(2.5),
                    transitions={'success': 'STABILIZE', 'fail': 'fail'})

            smach.StateMachine.add('STABILIZE', stabilize(),
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

        hsv_im = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        red_low_hues = cv2.inRange(hsv_im, lower_red, upper_red)

        lower_red = np.array([160, 100, 100])
        upper_red = np.array([179, 255, 255])
        red_high_hues = cv2.inRange(hsv_im, lower_red, upper_red)
        red_filtered = cv2.addWeighted(red_high_hues, 1.0, red_low_hues, 1.0, 0)
        blurred = cv2.GaussianBlur(red_filtered, (9,9), 0)
        im, contours, hierarchy = cv2.findContours(blurred, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        sorted_contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
        points = np.concatenate(sorted_contours[:1])
        (x, y), radius = cv2.minEnclosingCircle(points)

        min_x_pt = (im.shape[1], 0)
        max_x_pt = (0, 0)
        max_y_pt = (0, 0)
        min_y_pt = (0, im.shape[0])

        for point in points:
            point = point.flatten()
            if point[0] < min_x_pt[0]:
                min_x_pt = point

            if point[0] > max_x_pt[0]:
                max_x_pt = point

            if point[1] < min_y_pt[1]:
                min_y_pt = point

            if point[1] > max_y_pt[1]:
                max_y_pt = point

        left_slice = [(x, y), min_x_pt, (0, 0)]
        right_slice = [(x, y), max_x_pt, (0, 0)]

        diff_min_y = map(sub, min_x_pt, min_y_pt)
        diff_max_y = map(sub, min_x_pt, max_y_pt)
        d_min_y = np.sqrt(diff_min_y[0]**2 + diff_min_y[1]**2)
        d_max_y = np.sqrt(diff_max_y[0]**2 + diff_max_y[1]**2)
        if d_min_y < d_max_y:
            left_slice[2] = min_y_pt
            right_slice[2] = max_y_pt
        else:
            left_slice[2] = max_y_pt
            right_slice[2] = min_y_pt

        left_added = (0, 0)
        right_added = (0, 0)
        for pt in left_slice:
            left_added = map(add, pt, left_added)
        for pt in right_slice:
            right_added = map(add, pt, right_added)

        left_slice_center = (int(left_added[0] / 3), int(left_added[1] / 3))
        right_slice_center = (int(right_added[0] / 3), int(right_added[1] / 3))

#        slice_contours = [left_slice]
#        cv2.drawContours(img, slice_contours, -1, (0, 255, 0), 3)
        rospy.loginfo('Point at left slice center: {}'.format(red_filtered[left_slice_center]))
        cv2.circle(img, left_slice_center, 3, (255, 0, 0), 3)
        cv2.imshow('Roulette wheel', img)
        cv2.waitKey(1)

        detection = Detection()
        detection.x = float(left_slice_center[0]) / im.shape[1]
        detection.y = float(left_slice_center[1]) / im.shape[0]

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


class drop_marker(smach.State):
    def __init__(self, color, outcomes, input_keys=[], output_keys=[]):
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)


    def execute(self, user_data):
        service = rospy.ServiceProxy('drop_marker', Empty)
        ret = service()
        return 'success' if ret == True else 'fail'


class roulette_task(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'])

        with self:
            smach.StateMachine.add('DIVE_PINGER', go_to_depth(1.5),
                    transitions={'success': 'FIND_PINGER', 'fail': 'fail'})

            smach.StateMachine.add('FIND_PINGER', move_to_pinger(),
                    transitions={'success': 'STABILIZE',
                                 'fail': 'fail'})

            smach.StateMachine.add('STABILIZE', stabilize(),
                    transitions={'success': 'LOCATE_ROULETTE', 'fail': 'fail'})

            smach.StateMachine.add('LOCATE_ROULETTE', center_above('roulette_wheel'),
                    transitions={'success': 'DIVE_TARGET',
                                 'fail': 'fail'})

            smach.StateMachine.add('DIVE_TARGET', go_to_depth(3),
                    transitions={'success': 'CENTER_TARGET', 'fail': 'fail'})

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
