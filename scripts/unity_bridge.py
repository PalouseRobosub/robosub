#!/usr/bin/env python

import signal
import json
from math import radians

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

# Clean up zmq before ros shutdown
def exitHandler(signum, frame):
    global sock
    global ctx
    sock.close()

    rospy.signal_shutdown(signum)

# Convert thruster msg to json and send to unity
def thrusterCallback(msg):
    global sock

    # Converting ros thruster message to legacy thruster msg
    # then packaging that in legacy message type
    thruster_dict = message_converter.convert_ros_message_to_dictionary(msg)

    value = {}
    for i in range(len(thruster_dict["data"])):
        value[str(i)] = thruster_dict["data"][i]
    outmsg = {"sender": "unity_bridge", "recipient": "thruster", "value":str(value), "mtype": "thruster"}

    #print json.dumps(outmsg)

    try:
        sock.send_multipart(["thruster", json.dumps(outmsg)], flags=zmq.NOBLOCK)
    except zmq.Again as e:
        pass

# Convert sensor msg (json) from unity to one or more ros msgs
def sensorZMQCallback(msg):
    # load json into dictionary
    d = json.loads(msg)
    # Acual message is stored in value field so reparse
    if d["mtype"] == "sensor":
        value = json.loads(d["value"])

        # Extract rpy and convert to quaternion
        roll = value["roll"]
        pitch = value["pitch"]
        yaw = value["yaw"]
        print "roll: ",
        print roll
        print "pitch: ",
        print pitch
        print "yaw: ",
        print yaw
        print
        quat = tf.transformations.quaternion_from_euler(radians(roll), radians(pitch), radians(yaw), axes="sxyz")

        # Extract depth
        depth = value["depth"]

        # Convert tf quat to rosmsg quat
        quat_msg = Quaternion()
        quat_msg.x = quat[0]
        quat_msg.y = quat[1]
        quat_msg.z = quat[2]
        quat_msg.w = quat[3]

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

    ctx = zmq.Context()
    sock = ctx.socket(zmq.ROUTER)
    sock.set(zmq.IDENTITY, "broker")
    sock.bind("tcp://127.0.0.1:2222")

    rospy.init_node('unity_bridge')
    rospy.Subscriber('thruster', thruster, thrusterCallback)
    orientation_pub = rospy.Publisher('orientation', Quaternion, queue_size=1)
    depth_pub = rospy.Publisher('depth', Float32, queue_size=1)

    rate = rospy.get_param("unity_bridge/rate", RATE_DEFAULT)
    r = rospy.Rate(rate)

    while not rospy.is_shutdown():
        msg = ""
        try:
            identity, msg = sock.recv_multipart(flags=zmq.NOBLOCK)
            sensorZMQCallback(msg)
        except zmq.Again as e:
            pass
            #print e

        r.sleep()
        # No need for spin or spinOnce here since
        # Subscriber callbacks are executed in another thread

if __name__=="__main__":
    main()
