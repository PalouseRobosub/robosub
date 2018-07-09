#!/usr/bin/python
"""
@author Ryan Summers
@date 2-28-2018

@brief Provides a bridge from the raw socket API implemented on the HydroZynq to
       the ROS infrastructure.
"""

import argparse
import re
import rospy
import socket
import struct
import threading

from std_srvs.srv import SetBool
from robosub_msgs.msg import HydrophoneDeltas
from robosub.srv import SetInt, SetIntResponse, SetIntRequest


class DeltaPacket:
    """ Parses a string for the hydrophone delta result.

    Attributes
        x: The delay in seconds of the first channel.
        y: The delay in seconds of the second channel.
        z: The delay in seconds of the third channel.
    """

    def __init__(self, data):
        # The result string is human readable in the form:
        #   Result - 1: [time in ns] 2: [time in ns] 3: [time in ns]
        matches = re.search(r'.*1: (-?\d+) 2: (-?\d+) 3: (-?\d+).*', data)
        if not matches:
            raise Exception('Invalid result string')

        if len(matches.groups()) != 3:
            raise Exception('Valid number of groups')

        # Convert the nanosecond results into seconds.
        self.x = float(matches.group(1)) / 1000000000.0
        self.y = float(matches.group(2)) / 1000000000.0
        self.z = float(matches.group(3)) / 1000000000.0


class HydroNode:
    """ Handles bridge tasks for the hydrophone node.

    Attributes
        running: Boolean that specifies true if the threads should continue.
        hostname: The hostname of Cobalt
        zynq_hostname: The hostname of the Zynq processor.
        delta_pub: A ROS publisher for hydrophone delays.
        silence_thread: A thread for silencing the control system.
        stdout_thread: A thread for reading the Zynq STDOUT.
        result_thread: A thread for reading the Zynq hydrophone delays.
        ping_threshold_service: A thread for setting the Zynq ping threshold.
    """

    def __init__(self, hostname, zynq_hostname):
        """ Initializes the node.

        Args
            hostname: The hostname of the current computer.
            zynq_hostname: The hostname of the Zynq processor.
        """
        self.running = False
        self.hostname = hostname
        self.zynq_hostname = zynq_hostname
        self.delta_pub = rospy.Publisher(
                'hydrophones/30khz/delta', HydrophoneDeltas, queue_size=10)

        self.silence_thread = threading.Thread(
                target=self._silence_thread_target,
                name='Control Silencer Thread')

        self.stdout_thread = threading.Thread(
                target=self._stdout_thread_target,
                name='ROS STDOUT Thread')

        self.result_thread = threading.Thread(
                target=self._result_thread_target,
                name='Result Thread')

        self.ping_threshold_service = rospy.Service(
                'hydrophone_bridge/set_ping_threshold',
                SetInt,
                self.set_threshold)


    def begin(self):
        """ Start all threads. """
        self.running = True
        self.silence_thread.start()
        self.stdout_thread.start()
        self.result_thread.start()


    def end(self):
        """ Stop all threads. """
        self.running = False
        rospy.loginfo('Terminating threads...')
        self.silence_thread.join()
        self.stdout_thread.join()
        self.result_thread.join()


    def set_threshold(self, req):
        """ ROS Service callback for setting ping threshold. """
        arg_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        arg_sock.connect((self.zynq_hostname, 3000))

        command = 'threshold:{}'.format(req.data)
        arg_sock.send(command)
        arg_sock.close()
        return SetIntResponse(True)


    def _silence_thread_target(self):
        """ Handles requests for silencing thrusters. """
        rospy.loginfo('BRIDGE: Waiting for control/silence service.')
        rospy.wait_for_service('control/silence')
        rospy.loginfo('BRIDGE: Service acquired.')
        control_shutdown_srv = rospy.ServiceProxy('control/silence', SetBool)

        silence_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        silence_sock.settimeout(0.5)
        silence_sock.bind((self.hostname, 3005))

        while not rospy.is_shutdown() and self.running:
            try:
                data = silence_sock.recv(1024)
            except socket.timeout:
                continue

            recv_time = rospy.get_time()
            if len(data) != 8:
                rospy.logwarn('Received invalid packet size.')
                continue

            when, duration = struct.unpack('<ii', data)


            while rospy.get_time() < recv_time + (when / 1000.0):
                continue

            rospy.loginfo('Silencing thrusters for ping.')
            control_shutdown_srv(True)

            shutdown_time = rospy.get_time()
            while rospy.get_time() < shutdown_time + (duration / 1000.0):
                continue

            control_shutdown_srv(False)
            rospy.loginfo('Disabling silence')
        rospy.loginfo('Silence thread terminating...')

        silence_sock.close()


    def _stdout_thread_target(self):
        """ Handles displaying the Zynq standard output. """
        stdout_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        stdout_sock.settimeout(0.5)
        stdout_sock.bind((self.hostname, 3004))

        while not rospy.is_shutdown() and self.running:
            try:
                line = stdout_sock.recv(1024).rstrip('\n')
            except socket.timeout:
                continue

            rospy.loginfo(line)
        rospy.loginfo('STDOUT thread terminating...')
        stdout_sock.close()


    def _result_thread_target(self):
        """ Handles receiving the Zynq's hydrophone delay results. """
        result_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        result_sock.settimeout(0.5)
        result_sock.bind((self.hostname, 3002))

        while not rospy.is_shutdown() and self.running:
            try:
                data = result_sock.recv(1024)
            except socket.timeout:
                continue

            try:
                deltas = DeltaPacket(data)
            except Exception as e:
                rospy.logwarn(
                        'Received invalid HydroZynq datagram: {}'.format(e))
                continue

            msg = HydrophoneDeltas()

            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'hydrophone_array'
            msg.xDelta = rospy.Duration.from_sec(deltas.x)
            msg.yDelta = rospy.Duration.from_sec(deltas.y)
            msg.zDelta = rospy.Duration.from_sec(deltas.z)

            self.delta_pub.publish(msg)
        rospy.loginfo('Result thread terminating...')
        result_sock.close()


if __name__ == '__main__':
    rospy.init_node('hydrophone_bridge')

    hostname = rospy.get_param('~hostname', default='192.168.0.2')
    zynq_hostname = rospy.get_param('~zynq_hostname', default='192.168.0.7')
    threshold = rospy.get_param('~threshold', default=500)

    node = HydroNode(hostname, zynq_hostname)

    node.begin()

    node.set_threshold(SetIntRequest(threshold))

    rospy.spin()
    node.end()
