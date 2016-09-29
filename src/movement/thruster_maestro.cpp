#include "ros/ros.h"
#include "utility/serial.hpp"
#include "robosub/thruster.h"

using namespace rs;
Serial outmessage;

uint16_t translatemessage (const double data)
{
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
		if(!n.getParam("thruster_serial_port", thruster_port))
    {
      ROS_FATAL("no serial port specified, exiting!");
      exit(1);
    }
		outmessage.Open(thruster_port.c_str(), B9600);
		ros::spin();
    return 0;
}
