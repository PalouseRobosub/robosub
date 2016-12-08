//This file is designed to test the serial_subscriber.cpp example, which
//receives messages of the robosub/thruster type, and then sends the exact
//received value through a serial port. This example sends a single thruster
//message with several thrusters inside, and then listens on a serial port to
//confirm that the received values are the same as the sent values.
#include <gtest/gtest.h>
#include "ros/ros.h"
#include "robosub/thruster.h"
#include "utility/serial.hpp"
#include "utility/test_tools.hpp"

rs::Serial mSerial;
ros::Publisher pub;
double byte_check(uint8_t byte2, uint8_t byte3);
std::vector<uint8_t> channels;
double max_speed = 0.6;

//insert tests here
TEST(Maestro, basicTest)
{
    //allocate an array of bytes that will be used to receive serial data from
    //the serial port. I use the arbitrarily large size of
    uint8_t maestro_data[256] = {0};


    //create an empty thruster message to send to the thruster module
    robosub::thruster maestro_msg;
    for (uint8_t i = 0; i < channels.size(); i++)
    {
        maestro_msg.data.push_back(0.0);
    }
    pub.publish(maestro_msg);
    ros::Duration(2).sleep();
    mSerial.Flush();

    maestro_msg.data.clear();


    //fill out data in the thruster message.
    maestro_msg.data.push_back(1.0);
    maestro_msg.data.push_back(0.7);
    maestro_msg.data.push_back(0.8);
    maestro_msg.data.push_back(-0.5);
    maestro_msg.data.push_back(-0.8);
    maestro_msg.data.push_back(-1.0);

    //publish the thruster message
    pub.publish(maestro_msg);

    ROS_INFO("published message, hit enter to read serial");
    ros::Duration(1).sleep();

    //after we published the thruster message, the thruster module should have
    //received the message, and then sent some data down the serial port, here
    //we read that data and verify it is correct
    for(unsigned int i = 0; i < maestro_msg.data.size(); ++i)
    {


        //read one byte from the serial port
        int bytes_rxd = mSerial.Read(maestro_data, 4);
        ROS_INFO("recieved %d bytes", bytes_rxd);
        //mSerial.Read(maestro_data, 4);

        ROS_WARN("raw_vals: %x, %x", maestro_data[2], maestro_data[3]);
        double maestro_decimal = byte_check(maestro_data[2], maestro_data[3]);


        //checks to make sure the first byte in maestro_data is correct
        EXPECT_EQ(maestro_data[0], 0x84);
        EXPECT_EQ(maestro_data[1], channels[i]);

        //compare and make sure the value recieved is equal to the value sent
        double speed = maestro_msg.data[i];
        if(fabs(speed) > max_speed)
        {
            speed = max_speed * ((speed < 0)? -1 : 1);
        }
        EXPECT_EQ(maestro_decimal, speed);


    }
    ROS_INFO("test over, waiting");
}

double byte_check(uint8_t byte2, uint8_t byte3)
{
  double target;
  uint16_t pulse_width;

  pulse_width = ((byte3 << 7) | byte2) >> 2;
  target = (double)(pulse_width - 1500) / 400;

  return target;
}


int main(int argc, char *argv[])
{
    //initialize the google testing framework
    testing::InitGoogleTest(&argc, argv);

    //initialize our ros node
    ros::init(argc, argv, "maestro_launch_test");
    ros::NodeHandle n;

    XmlRpc::XmlRpcValue my_list;
    ros::param::get("thrusters", my_list);
    for (int i = 0; i < my_list.size(); ++i)
    {
        channels.push_back( static_cast<int>(my_list[i]["channel"]));
    }

    //SerialTB (serial testbench) creates 2 virtual serial ports and links them
    //to gether. We open one port, and the UUT opens the other port
    rs::SerialTB serial_tb;

    //variables to store the names of the serial ports, SerialTB.Start() will
    //fill them out
    std::string UUT_port;
    std::string testing_port;

    //try to start the serial testbench. If successful, it will return 0, on
    //failure it will return 1
    if(serial_tb.Start(testing_port, UUT_port) != 0)
    {
        ROS_FATAL("could not start serial testbench, failing test");
        exit(1);
    }

    //set the value of the thruster_serial_port parameter, the UUT will read
    //this variable so it knows what serial port to open
    ros::param::set("ports/thruster", UUT_port);

    //open our serial port
    mSerial.Open(testing_port.c_str(), B9600);

    //initialize our publisher used for sending thruster messages
    pub = n.advertise<robosub::thruster>("thruster", 1);

    ROS_INFO("all setup, press enter");
    //ros::Duration(4).sleep();
    /////getchar();
    //wait for the UUT to finish launching, can't run tests until the UUT has
    //set itself up. This function will wait for 3 seconds for the UUT to
    //initialize, then it will error out and fail the test
    ROS_INFO("waiting for subscriber");
    rs::wait_for_subscriber(pub, 5);
    ROS_INFO("done!");

    return RUN_ALL_TESTS();
}
