#include <gtest/gtest.h>
#include "ros/ros.h"
#include "robosub/thruster.h"
#include "utility/serial_testbench.hpp"
#include "utility/serial.hpp"

typedef struct thruster_info
{
  std::string name;
  uint8_t address;
  uint8_t port;      // 0 or 1
}Thruster_info;

Thruster_info *mThruster_info;
Serial mSerial;

TEST(SerialSubscriber, basicTest)
{
    //Variables for testing use.
    uint8_t address = 0, cmd_byte = 0, speed = 0, checksum = 0;
    int count = 0;
    //allocate an array of bytes that will be used to receive serial data from
    //the serial port. I use the arbitrarily large size of
    uint8_t serial_data[256] = {0};

    //create an empty thruster message to send to the thruster module
    robosub::thruster saber_msg;

    //fill out data in the thruster message.
    saber_msg.data.push_back(1);
    saber_msg.data.push_back(-1);
    saber_msg.data.push_back(0.5);
    saber_msg.data.push_back(0.1);
    saber_msg.data.push_back(-0.5);
    saber_msg.data.push_back(-0.1);

    //publish the thruster message
    pub.publish(saber_msg);

    //after we published the thruster message, the thruster module should have
    //received the message, and then sent some data down the serial port, here
    //we read that data and verify it is correct
    //read one byte from the serial port

    //
    while(count < 6)
    {
        /*
        if(count < 2)
        {
          address = 128;
        }
        else if(count < 4)
        {
          address = 129;
        }
        else
        {
          address = 130;
        } */

        //Gets address
        address = mThruster_info[count].address

        //If positive, command should be 0
        if(saber_msg.data[count] > 0)
        {
          cmd_byte = 0;
        }

        //If negative, command should be 1
        else if(saber_msg.data[count] < 0)
        {
          cmd_byte = 1;
        }

        //Calculates speed
        speed = abs(saber_msg.data[count] * 127);

        //Check sum
        checksum = address + cmd_byte + speed;

        mSerial.Read(serial_data, 4);

        //compare and make sure the value recieved is equal to the value sent
        EXPECT_EQ(address, serial_data[0]);
        EXPECT_EQ(cmd_byte, serial_data[1]);
        EXPECT_EQ(speed, serial_data[2]);
        EXPECT_EQ(checksum, serial_data[3]);

        count++;
    }
}

int main(int argc, char *argv[])
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "sabertooth_thruster_test");
    ros::NodeHandle n;

  rs::SerialTB serial_tb;

  std::string UUT_port;
  std::string testing_port;

  if(serial_tb.Start(testing_port, UUT_port) != 0)
  {
        ROS_FATAL("could not start serial testbench, failing test");
        exit(1);
  }

  ros::param::set("thruster_serial_port", UUT_port);
  mSerial.Open(testing_port.c_str(), B9600);

  pub = n.advertise<robosub::thruster>("thruster", 1);

  rs::wait_for_subscriber(pub, 3);
  return RUN_ALL_TESTS();
}