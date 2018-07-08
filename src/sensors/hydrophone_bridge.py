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
from robosub_msgs.msg import HydrophoneDeltas, HydrophoneStatus
from robosub.srv import SetInt, SetIntResponse, SetIntRequest


class DeviceStatus:
    """ Parses a string for hydrophone status information.

    Attributes:
        firmware_revision: The firmware revision of the device.
        on_time: The total time (in seconds) that the device has been on.
        sampling_frequency: The sampling frequency of the analog signals in Hz.
        primary_frequency: The primary ping frequency tracked.
        device_temp: The temperature of the chip in degrees F.
        ping_frequency: The frequency of the most recent ping measurement.
        record_time: The number of seconds that the last recording took.
        fft_time: The number of seconds the last FFT took.
        correlation_time: The number of seconds the last correlation took.
        ping_processing_time: The total number of seconds to process the last ping.
        tracking_25: True if 25KHz is being tracked.
        tracking_30: True if 30KHz is being tracked.
        tracking_35: True if 35KHz is being tracked.
        tracking_40: True if 40KHz is being tracked.
        detection_threshold: The current detection threshold value.
    """

    def __init__(self, data):
        self.data = data

        match = re.search('Firmware Revision: (?P<rev>.*)\n'
                     'On Time: (?P<time>.*?) s\n'
                     'Sampling Frequency: (?P<sample_rate>\d+) Hz\n'
                     'Primary Frequency: (?P<primary_freq>\d+)\n'
                     'Fpga Temp: (?P<temp>\d+) \*F\n'
                     'Ping Frequency: (?P<freq>\d+) Hz\n'
                     'Record \+ Normalize Time: (?P<record>\d+) us\n'
                     'FFT Time: (?P<fft>\d+) us\n'
                     'Correlation Time: (?P<correlate>\d+) us\n'
                     'Ping processing Time: (?P<ping_time>\d+) us\n'
                     'Tracking 25KHz: (?P<track25>\d+)\n'
                     'Tracking 30KHz: (?P<track30>\d+)\n'
                     'Tracking 35KHz: (?P<track35>\d+)\n'
                     'Tracking 40KHz: (?P<track40>\d+)\n'
                     'Detection Threshold: (?P<threshold>\d+)\n', data, re.MULTILINE)

        if not match:
            raise Exception('Failed to match status message.')

        self.firmware_revision = match.group('rev')
        self.on_time = float(match.group('time'))
        self.sampling_frequency = int(match.group('sample_rate'))
        self.primary_frequency = int(match.group('primary_freq'))
        self.device_temp = int(match.group('temp'))
        self.ping_frequency = int(match.group('freq'))
        self.record_time = float(match.group('record')) / 1000000.0
        self.fft_time = float(match.group('fft')) / 1000000.0
        self.correlation_time = float(match.group('correlate')) / 1000000.0
        self.ping_processing_time = float(match.group('ping_time')) / 1000000.0
        self.tracking_25 = match.group('track25') == '1'
        self.tracking_30 = match.group('track30') == '1'
        self.tracking_35 = match.group('track35') == '1'
        self.tracking_40 = match.group('track40') == '1'
        self.detection_threshold = int(match.group('threshold'))


    def as_message(self):
        msg = HydrophoneStatus()
        msg.revision = self.firmware_revision
        msg.on_time = rospy.Time.from_sec(self.on_time)
        msg.sampling_frequency = self.sampling_frequency
        msg.primary_frequency = self.primary_frequency
        msg.temp = self.device_temp
        msg.frequency = self.ping_frequency
        msg.record_time = rospy.Time.from_sec(self.record_time)
        msg.fft_time = rospy.Time.from_sec(self.fft_time)
        msg.correlation_time = rospy.Time.from_sec(self.correlation_time)
        msg.processing_time = rospy.Time.from_sec(self.ping_processing_time)
        msg.track_25khz = self.tracking_25
        msg.track_30khz = self.tracking_30
        msg.track_35khz = self.tracking_35
        msg.track_40khz = self.tracking_40
        msg.threshold = self.detection_threshold

        return msg


class DeltaPacket:
    """ Parses a string for the hydrophone delta result.

    Attributes
        f: The frequency (in KHz) that the packet corresponds to.
        x: The delay in seconds of the first channel.
        y: The delay in seconds of the second channel.
        z: The delay in seconds of the third channel.
    """

    def __init__(self, data):
        # The result string is human readable in the form:
        # [num] KHz Result - 1: [time in ns] 2: [time in ns] 3: [time in ns]
        matches = re.search(r'(\d+) KHz Result - 1: (-?\d+) 2: (-?\d+) 3: (-?\d+).*', data)
        if not matches:
            raise Exception('Invalid result string')

        if len(matches.groups()) != 4:
            raise Exception('Invalid number of groups')

        # Convert the nanosecond results into seconds.
        self.f = int(matches.group(1))
        self.x = float(matches.group(2)) / 1000000000.0
        self.y = float(matches.group(3)) / 1000000000.0
        self.z = float(matches.group(4)) / 1000000000.0


class HydroNode:
    """ Handles bridge tasks for the hydrophone node.

    Attributes
        running: Boolean that specifies true if the threads should continue.
        hostname: The hostname of Cobalt
        zynq_hostname: The hostname of the Zynq processor.
        delta_pub: A ROS publisher for hydrophone delays.
        status_pub: A ROS publisher for zynq device status.
        silence_thread: A thread for silencing the control system.
        stdout_thread: A thread for reading the Zynq STDOUT.
        result_thread: A thread for reading the Zynq hydrophone delays.
        status_thread: A thread for reading the Zynq device status.
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

        # Set up the publishers for each frequency.
        self.delta_pub = dict()
        for i in [25, 30, 35, 40]:
            self.delta_pub[str(i)] = rospy.Publisher(
                    'hydrophones/{}khz/delta'.format(i),
                    HydrophoneDeltas,
                    queue_size=10)

        self.status_pub = rospy.Publisher('hydrophones/device_status',
                                          HydrophoneStatus,
                                          queue_size=10)

        self.silence_thread = threading.Thread(
                target=self._silence_thread_target,
                name='Control Silencer Thread')

        self.stdout_thread = threading.Thread(
                target=self._stdout_thread_target,
                name='ROS STDOUT Thread')

        self.result_thread = threading.Thread(
                target=self._result_thread_target,
                name='Result Thread')

        self.status_thread = threading.Thread(
                target=self._status_thread_target,
                name='Status Thread')

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
        self.status_thread.start()


    def end(self):
        """ Stop all threads. """
        self.running = False
        rospy.loginfo('Terminating threads...')
        self.silence_thread.join()
        self.stdout_thread.join()
        self.result_thread.join()
        self.status_thread.join()


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


    def _status_thread_target(self):
        """ Handles receiving the Zynq device status information for logging. """
        status_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        status_sock.settimeout(0.5)
        status_sock.bind((self.hostname, 3007))

        while not rospy.is_shutdown() and self.running:
            try:
                line = status_sock.recv(2048)
            except socket.timeout:
                continue

            # Process the device status information.
            try:
                parser = DeviceStatus(line)
            except Exception as e:
                rospy.loginfo('Could not parse device status \'{}\': {}'.format(line, e))
                continue

            # Construct the message and publish it.
            self.status_pub.publish(parser.as_message())

        rospy.loginfo('Status thread terminating...')
        status_sock.close()


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

            freq = str(deltas.f)
            if freq not in list(self.delta_pub.keys()):
                rospy.logerr('Got invalid frequency: {} KHz'.format(deltas.f))
                continue

            # Publish the delta message on the appropriate frequency publisher.
            self.delta_pub[freq].publish(msg)

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
