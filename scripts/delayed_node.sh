#!/bin/bash
# this script is used for delaying the launching of nodes within a launch file
# the first parameter is the delay time (in seconds), the second is the executable name
# usage example:
# <node name="serial_subscriber" pkg="robosub" type="delayed_node.sh" args="1 serial_subscriber"/>
sleep $1
echo "got here"
rosrun robosub $2
echo "returned here"
