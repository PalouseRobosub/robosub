//This file is designed to test the serial_subscriber.cpp example, which
//receives messages of the robosub/thruster type, and then sends the exact
//received value through a serial port. This example sends a single thruster
//message with several thrusters inside, and then listens on a serial port to
//confirm that the received values are the same as the sent values.
#include <gtest/gtest.h>
#include "ros/ros.h"
#include "robosub_msgs/thruster.h"
#include "utility/serial.hpp"
#include "utility/test_tools.hpp"
#include <string>

//some global variables used by multiple tests
rs::Serial mSerial;
ros::Publisher pub;


//insert tests here
TEST(SerialSubscriber, basicTest)
{
    //allocate an array of bytes that will be used to receive serial data from
    //the serial port. I use the arbitrarily large size of
    uint8_t serial_data[256] = {0};

    //create an empty thruster message to send to the thruster module
    robosub_msgs::thruster thruster_msg;

    //fill out data in the thruster message.
    thruster_msg.data.push_back(27);
    thruster_msg.data.push_back(32);
    thruster_msg.data.push_back(59);
    thruster_msg.data.push_back(10);

    //publish the thruster message
    pub.publish(thruster_msg);

    //after we published the thruster message, the thruster module should have
    //received the message, and then sent some data down the serial port, here
    //we read that data and verify it is correct
    for(unsigned int i = 0; i < thruster_msg.data.size(); ++i)
    {
        //read one byte from the serial port
        mSerial.Read(serial_data, 1);

        //compare and make sure the value recieved is equal to the value sent
        EXPECT_EQ(thruster_msg.data[i], serial_data[0]);
    }
}

int main(int argc, char *argv[])
{
    //initialize the google testing framework
    testing::InitGoogleTest(&argc, argv);

    //initialize our ros node
    ros::init(argc, argv, "serial_subscriber_test");
    ros::NodeHandle n;


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
    ros::param::set("thruster_serial_port", UUT_port);

    //open our serial port
    mSerial.Open(testing_port.c_str(), B9600);

    //initialize our publisher used for sending thruster messages
    pub = n.advertise<robosub_msgs::thruster>("thruster", 1);

    //wait for the UUT to finish launching, can't run tests until the UUT has
    //set itself up. This function will wait for 3 seconds for the UUT to
    //initialize, then it will error out and fail the test
    rs::wait_for_subscriber(pub, 3);

    return RUN_ALL_TESTS();
}
