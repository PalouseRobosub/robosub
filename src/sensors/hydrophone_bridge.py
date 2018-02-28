#!/usr/bin/python

import argparse
import re
import rospy
import socket
import struct

from std_srvs.srv import SetBool
from robosub_msgs.msg import HydrophoneDeltas


class DeltaPacket:
    def __init__(self, data):
        matches = re.search(r'.*1: (-?\d+) 2: (-?\d+) 3: (-?\d+).*', data)
        if not matches:
            raise Exception('Invalid result string')

        if len(matches.groups()) != 3:
            raise Exception('Valid number of groups')

        self.x = int(matches.group(1))
        self.y = int(matches.group(2))
        self.z = int(matches.group(3))


class HydroNode:
    def __init__(self, hostname):
        self.running = False
        self.hostname = hostname
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


    def begin(self):
        self.running = True
        self.silence_thread.start()
        self.stdout_thread.start()
        self.result_thread.start()


    def end(self):
        self.running = False
        self.silence_thread.join(1.0)
        self.stdout_thread.join(1.0)
        self.result_thread.join(1.0)


    def _silence_thread_target(self):
        silence_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        silence_sock.bind((self.hostname, 3005))

        control_shutdown_srv = rospy.ServiceProxy('control/silence', SetBool)

        while self.running:
            data = silence_sock.recv(1024)
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
        silence_sock.close()


    def _stdout_thread_target(self):
        stdout_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        stdout_sock.bind((self.hostname, 3004))

        while self.running:
            rospy.loginfo(sock.recv(1024).rstrip('\n'))
        stdout_sock.close()


    def _result_thread_target(self):
        result_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        result_sock.bind((self.hostname, 3002))

        while self.running:
            data = sock.recv(1024)

            try:
                deltas = DeltaPacket(data)
            except Exception as e:
                rospy.logwarn(
                        'Received invalid HydroZynq datagram: {}'.format(e))
                continue

            msg = HydrophoneDeltas()

            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'hydrophone_array'
            msg.xDelta = rospy.Duration(deltas.x)
            msg.yDelta = rospy.Duration(deltas.y)
            msg.zDelta = rospy.Duration(deltas.z)

            self.delta_pub.publish(msg)
        result_sock.close()


if __name__ == '__main__':
    rospy.init_node('hydrophone_bridge')

    hostname = rospy.get_param('~hostname', default='192.168.0.2')

    node = HydroNode(hostname)

    node.begin()
    rospy.spin()
    node.end()
