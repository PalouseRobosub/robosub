#include "ros/ros.h"
#include "utility/serial.hpp"
#include "robosub/thruster.h"
#include <string>
#include <vector>

using namespace rs;

typedef struct thruster_info
{
  std::string name;
  uint8_t address;
  uint8_t port;      // 0 or 1
}Thruster_info;

std::vector<Thruster_info> mThruster_info;
Serial mSerial;


void checksum (uint8_t serial_data[4])
{
  //sum the address, command, and data bytes and AND with 127
  serial_data[3] = (serial_data[0]+serial_data[1]+serial_data[2]) & 127;
}

/*
THRUSTER PACKET
serial_data[0] address
serial_data[1] command
serial_data[2] data bytes
serial_data[3] checksum (call checksum function)
*/
void createThrusterPacket (uint8_t serial_data[4], double value, int index)
{
    serial_data[0] = mThruster_info[index].address;

    /*Command*/
    //Port 1
    if(mThruster_info[index].port == 0)
    {
      if(value < 0)
      { //command - backwards port 1
        serial_data[1] = 1;
      }
      else
      { //command - forwards port 1
        serial_data[1] = 0;
      }
    }
    //Other ports (Port 2)
    else
    {
      if(value < 0)
      { //command - backwards port 2
        serial_data[1] = 5;
      }
      else
      { //command - forwards port 2
        serial_data[1] = 4;
      }
    }

    //Check for NaNs
    if (value != value)
    {
        serial_data[2] = 0;
        ROS_ERROR("NaN recieved. Stopping thruster.");
    }
    else
    {
      //data (speed) value between 0 - 127
      serial_data[2] = abs(value * 127);
    }

    //checksum
    checksum(serial_data);
}

/*Message recieved, send thruster package to thrusters
  Publish thruster information with name*/
void callBack(const robosub::thruster::ConstPtr& msg)
{
    robosub::thruster message = *msg;
    uint8_t serial_data[4];

    for (unsigned int i = 0; i < message.data.size(); ++i)
    {
      createThrusterPacket(serial_data, message.data[i], i);

      //send package to thrusters
      mSerial.Write(serial_data, 4);

      //Publish thruster info with name
      ROS_DEBUG_STREAM(mThruster_info[i].name << ":\t" << message.data[i]);
    }
}

/*Set a timeout
  Hardware cannot have a value greater than 256: uint8_t*/
void setTimeOut (uint8_t ms_100)
{
  uint8_t serial_data[4];

  //loop through thrusters
  for (unsigned int i = 0; i < mThruster_info.size(); i++)
  {
    //adress: loop over addresses
    serial_data[0] = mThruster_info[i].address;
    //command
    serial_data[1] = 14; //serial timeout
    //timeout value
    serial_data[2] = ms_100;
    //checksum
    checksum(serial_data);

    //send package to thrusters
    mSerial.Write(serial_data, 4);
  }
  //Timeout message
  ROS_INFO("Setting Timeout");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "thruster");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("thruster", 1, callBack);


    std::string thruster_port;
    if(!n.getParam("ports/thruster", thruster_port))
    {
        ROS_FATAL("No serial port specified!");
        exit(1);
    }

    mSerial.Open(thruster_port.c_str(), B9600);

    XmlRpc::XmlRpcValue my_list;
    ros::param::get("thrusters", my_list);

    for(int i = 0; i < my_list.size(); ++i)
    {
        ROS_DEBUG_STREAM("thrusters["<< i << "][name]:    " <<
                         my_list[i]["name"]);
        ROS_DEBUG_STREAM("thrusters["<< i << "][address]: " <<
                         my_list[i]["address"]);
        ROS_DEBUG_STREAM("thrusters["<< i << "][port]:    " <<
                         my_list[i]["port"]);
        Thruster_info one_thruster;
        one_thruster.name = static_cast<std::string>(my_list[i]["name"]);
        one_thruster.address = static_cast<int>(my_list[i]["address"]);
        one_thruster.port = static_cast<int>(my_list[i]["port"]);
        mThruster_info.push_back(one_thruster);
      }

  setTimeOut(5); //0.5 sec timeout

  ros::spin();

  //shutdown thrusters when node exits
  uint8_t serial_data[4];
  //Loop through thrusters
  for (unsigned int i = 0; i < mThruster_info.size(); i++)
  {
    createThrusterPacket(serial_data, 0, i);
    mSerial.Write(serial_data, 4);
  }

  return 0;
}