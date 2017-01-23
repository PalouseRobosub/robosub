#!/usr/bin/python

import rospy

import Queue

from Gyrus import *

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import String
from robosub.msg import visionPosArray
from robosub.msg import WeightedPosition
from robosub.msg import Localization

orientationQueue = Queue.Queue(maxsize=1000)
depthQueue = Queue.Queue(maxsize=1000)
redBuoyQueue = Queue.Queue(maxsize=1000)
greenBuoyQueue = Queue.Queue(maxsize=1000)
blueBuoyQueue = Queue.Queue(maxsize=1000)
magQueue = Queue.Queue(maxsize=1000)
accelQueue = Queue.Queue(maxsize=1000)

# publishes to our output
pub = rospy.Publisher("/localization", String, queue_size=10)

def sendBackXYZ():
    point = WeightedPosition()
    point.x = random.random()
    point.y = random.random()
    point.z = random.random()
    point.variance = random.random()

    msg = Localization()

    msg.data.append(point)

    pub.publish(msg)

def orientationCallBack(msg):
    print(msg)
    orientationQueue.put(msg)
    sendBackXYZ()
    return

def depthCallBack(msg):
    print(msg)
    depthQueue.put(msg)
    sendBackXYZ()
    return

def redBuoyVisionCallBack(msg):
    print(msg)
    redBuoyQueue.put(msg)
    sendBackXYZ()
    return

def greenBuoyVisionCallBack(msg):
    print(msg)
    greenBuoyQueue.put(msg)
    sendBackXYZ()
    return

def blueBuoyVisionCallBack(msg):
    print(msg)
    blueBuoyQueue.put(msg)
    sendBackXYZ()
    return

def accelCallBack(msg):
    print(msg)
    sendBackXYZ()
    return

def magnetometerCallBack(msg):
    print(msg)
    sendBackXYZ()
    return

topicNames = {
              "/orientation": (Quaternion, orientationCallBack),
              "/depth": (Float32, depthCallBack),
              "/vision/buoy/red": (visionPosArray, redBuoyVisionCallBack),
              "/vision/buoy/green": (visionPosArray, blueBuoyVisionCallBack),
              "/vision/buoy/blue": (visionPosArray, greenBuoyVisionCallBack)
             }

bagTopics = {
             "/rs_accel_data": (Vector3, accelCallBack),
             "/rs_bno_data": (Quaternion, orientationCallBack),
             "/rs_depth_data": (Float32, depthCallBack),
             "/rs_mag_data": (Vector3, magnetometerCallBack)
            }

if __name__ == "__main__":
    print("main()")

    rospy.init_node("LocalizationTopicReader")

    for topic in bagTopics:
        print("Subscribing to topic " + topic + " type: " +
              str(bagTopics[topic][0]))
        rospy.Subscriber(topic, bagTopics[topic][0], bagTopics[topic][1])

    rospy.spin()
