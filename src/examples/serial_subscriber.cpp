#include "ros/ros.h"
#include "robosub/thruster.h"
#include "utility/serial.hpp"
#include <string>

rs::Serial mSerial;

void callback(const robosub::thruster::ConstPtr& msg)
{
  uint8_t serial_data[256];

    for (unsigned int i = 0; i < msg->data.size(); ++i)
    {
      serial_data[i] = msg->data[i];
      ROS_DEBUG("serial data[%d]: %d", i, serial_data[i]);
    }

    mSerial.Write(serial_data, msg->data.size());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_talker");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("thruster", 1, callback);
    std::string thruster_port;


    if(!n.getParam("thruster_serial_port", thruster_port))
    {
      ROS_FATAL("no serial port specified, exiting!");
      exit(1);
    }

    mSerial.Open(thruster_port.c_str(), B9600);

    ros::spin();
}
