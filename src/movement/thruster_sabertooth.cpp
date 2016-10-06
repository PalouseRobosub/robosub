#include "ros/ros.h"
#include "utility/serial.hpp"
#include "robosub/thruster.h"

using namespace rs;
Serial s;

void checksum (unit8_t serial_data[4]) const
{
  //sum the address, command, and data bytes and AND with 127
  serial_data[4] = (serial_data[0]+serial_data[1]+serial_data[2]) & 127;
}

void callBack(const robosub::thruster::ConstPtr& msg)
{
    robosub::thruster message = *msg;
    unit8_t serial_data[4];

    //serial_data[0] - address
    //serial_data[1] - command
    //serial_data[2] - data (speed)
    //serial_data[3] - checksum

    for (int i = 0; i < message.data.size(); ++i)
    {
      //address

      //command
      if()
      {

      }
      else
      {

      }
      //data (speed) value between 0 - 127
      serial_data[2] = abs(message.data * 127);
      //checksum
      checksum(serial_data);
      //sends info, leave there for now?
      ROS_INFO("%lf", message.data[i]);
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thruster");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("thruster", 1, callBack);
	std::string thruster_port;
	if(!n.getParam("thruster_serial_port", thruster_port))
	{
		ROS_FATAL("No serial port specified!");
		exit(1);
	}

	mSerial.open(thruster_port.c_str(), B9600);
  //I think I'm supposed to add checks here

    ros::spin();

    return 0;
}
