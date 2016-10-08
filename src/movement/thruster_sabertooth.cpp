#include "ros/ros.h"
#include "utility/serial.hpp"
#include "robosub/thruster.h"

using namespace rs;

typedef struct thruster_info
{
  uint8_t address;
  uint8_t port;      // 0 or 1
}Thruster_info;

Thruster_info *mThruster_info;
Serial mSerial;


void checksum (uint8_t serial_data[4])
{
  //sum the address, command, and data bytes and AND with 127
  serial_data[3] = (serial_data[0]+serial_data[1]+serial_data[2]) & 127;
}

void callBack(const robosub::thruster::ConstPtr& msg)
{
    robosub::thruster message = *msg;
    uint8_t serial_data[4];

    //serial_data[0] - address
    //serial_data[1] - command
    //serial_data[2] - data (speed)
    //serial_data[3] - checksum

    for (int i = 0; i < message.data.size(); ++i)
    {
      //address
      serial_data[0] = mThruster_info[i].address;
      //command
      if(mThruster_info[i].port == 0)
      {
        if(message.data[i] < 0)
        { //command - backwards port 1
          serial_data[1] = 1;
        }
        else
        { //command - forwards port 1
          serial_data[1] = 0;
        }
      }
      else
      {
        if(message.data[i] < 0)
        { //command - backwards port 1
          serial_data[1] = 5;
        }
        else
        { //command - forwards port 1
          serial_data[1] = 4;
        }
      }

      //data (speed) value between 0 - 127
      serial_data[2] = abs(message.data[i] * 127);

      //checksum
      checksum(serial_data);

      //send package to thrusters
      mSerial.Write(serial_data, 4);

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

	mSerial.Open(thruster_port.c_str(), B9600);
  //I think I'm supposed to add checks here

  XmlRpc::XmlRpcValue my_list;
    ros::param::get("thrusters", my_list);

    mThruster_info = new struct thruster_info[my_list.size()];

    for(int i=0; i < my_list.size(); ++i)
    {
        ROS_DEBUG_STREAM("thrusters["<< i << "][name]:    " << my_list[i]["name"]);
        ROS_DEBUG_STREAM("thrusters["<< i << "][address]: " << my_list[i]["address"]);
        ROS_DEBUG_STREAM("thrusters["<< i << "][port]:    " << my_list[i]["port"]);
        mThruster_info[i].address = static_cast<int>(my_list[i]["address"]);
        mThruster_info[i].port = static_cast<int>(my_list[i]["port"]);
      }

  ros::spin();

  return 0;
}
