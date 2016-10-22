#!/usr/bin/env python

import signal
import json
from math import radians
from collections import OrderedDict

import rospy
import tf
from rospy_message_converter import message_converter
from geometry_msgs.msg import Quaternion
from robosub.msg import thruster
from robosub.msg import Euler
from std_msgs.msg import Float32

import zmq

RATE_DEFAULT = 100
orientation_pub = None
depth_pub = None
ctx = None
sock = None

"""
Unity Bridge provides a bridge service for converting messages from the legacy
unity simulator to ros. It basically emulates the legacy broker system
and then sends/receives messages as appropriate.

To go from simulator -> ros:
1. Define ros publisher on desired topic
2. Define function to handle serialized simulator message (see sensorZMQCallback)
3. Parse message as needed and convert to ros message
4. Publish with ros publisher

To go from ros -> simulator:
1. Define ros subscriber to recv desired topic (see thrusterCallback)
2. Use message_converter to convert ros message to python dict
3. Create legacy style message
4. Send to simulator using sock.send_multipart() providing the
   the legacy module identity and message (both strings)
"""

# Clean up zmq before ros shutdown
def exitHandler(signum, frame):
    global sock
    global ctx
    sock.close()

    rospy.signal_shutdown(signum)

#********************#
#  ROS -> Simulator  #
#********************#

# Convert thruster msg to json and send to unity
def thrusterCallback(msg):
    global sock

    # Converting ros thruster message to legacy thruster msg
    # then packaging that in legacy message type
    thruster_dict = message_converter.convert_ros_message_to_dictionary(msg)

    # Must use an OrderedDict since the simulator expects the thruster data
    # to be in same order as thrusters are in the thruster settings
    value = OrderedDict()
    for i in range(len(thruster_dict["data"])):
        value[str(i)] = thruster_dict["data"][i]
    outmsg = {"sender": "unity_bridge", "recipient": "thruster", "value":json.dumps(value), "mtype": "thruster"}

    try:
        sock.send_multipart(["thruster", json.dumps(outmsg)], flags=zmq.NOBLOCK)
    except zmq.Again as e:
        pass

#********************#
#  Simulator -> ROS  #
#********************#

# Convert sensor msg (json) from unity to one or more ros msgs
def sensorZMQCallback(msg):
    # load json into dictionary
    d = json.loads(msg)

    # Acual message is stored in value field so reparse
    if d["mtype"] == "sensor":
        value = json.loads(d["value"])

        # Extract depth
        depth = value["depth"]

        # Convert tf quat to rosmsg quat
        # Must convert unity coordinates to robosub coordinates
        quat_msg = Quaternion()
        quat_msg.x = -value["z"]
        quat_msg.y = value["x"]
        quat_msg.z = -value["y"]
        quat_msg.w = value["w"]

        # Make depth msg
        depth_msg = Float32()
        depth_msg.data = depth

        orientation_pub.publish(quat_msg)
        depth_pub.publish(depth_msg)

def main():
    global ctx
    global sock
    global orientation_pub
    global depth_pub
    signal.signal(signal.SIGINT, exitHandler)

    # Set up zmq socket.
    # Using a router socket (like the old broker) so that minimal changes on the
    # simulator side are necessary
    ctx = zmq.Context()
    sock = ctx.socket(zmq.ROUTER)
    sock.set(zmq.IDENTITY, "broker")
    sock.bind("tcp://127.0.0.1:2222")

    # Set up ros stuff
    rospy.init_node('unity_bridge')
    rospy.Subscriber('thruster', thruster, thrusterCallback)
    orientation_pub = rospy.Publisher('orientation', Quaternion, queue_size=1)
    depth_pub = rospy.Publisher('depth', Float32, queue_size=1)

    rate = rospy.get_param("unity_bridge/rate", RATE_DEFAULT)
    r = rospy.Rate(rate)

    while not rospy.is_shutdown():
        msg = ""
        try:
            # If receiving multiple types of messages from the simulator use
            # the identity to filter received messages by simulator module
            # (e.g sensors, hydrophone, etc)
            identity, msg = sock.recv_multipart(flags=zmq.NOBLOCK)
            if identity == "sensor":
                sensorZMQCallback(msg)
        except zmq.Again as e:
            pass

        # No need for spin or spinOnce here since
        # Subscriber callbacks are executed in another thread
        r.sleep()

if __name__=="__main__":
    main()
