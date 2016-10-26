#include "ros/ros.h"
#include "utility/serial.hpp"
#include "robosub/thruster.h"

using namespace rs;
Serial outmessage;

uint16_t translatemessage (const double data)
{
	/*
	To translate a value x in the range (a, b) into a value f(x) in the range (c, d)
	 f(x) = (x - a)*((d - c)/(b - a)) + c
	 For example: If we want to translate 0 in the interval (-1, 1) to it's equivalent in (1000, 2000)
	 {the value we want} = (0 - 1000)*((2000-1000)/(1 - (-1))) + 1000
	 */
	return 4*((data+1)*(500) + 1000);
}

void Callback (const robosub::thruster::ConstPtr& msg)
{
robosub::thruster message = *msg;
uint8_t serial_data[40];
for (int i = 0; i < message.data.size(); i++)
	{
		serial_data[0] = 132;
		serial_data[1] = i;
		uint16_t rawvalue = translatemessage(message.data[i]);
		serial_data[2] = rawvalue & 0xff;
		serial_data[3] = (rawvalue & 0xff00) >> 8;
	ROS_INFO("%lf", message.data[i]);
	outmessage.Write(serial_data, 4);
	}
}



int main(int argc, char **argv)
{
		std::string thruster_port;
    ros::init(argc, argv, "thruster");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("thruster", 1, Callback);
		if(!n.getParam("ports/thruster", thruster_port))
    {
      ROS_FATAL("no serial port specified, exiting!");
      exit(1);
    }
		outmessage.Open(thruster_port.c_str(), B9600);
		ros::spin();
    return 0;
}
