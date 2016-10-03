#include <gtest/gtest.h>
#include "ros/ros.h"
#include "robosub/thruster.h"
#include "utility/serial_testbench.hpp"
#include "utility/serial.hpp"
#include "utility/test_tools.hpp"

//some global variables
rs::Serial mSerial;
ros::Publisher pub;


//insert tests here
TEST(SerialSubscriber, basicTest)
{
    robosub::thruster thruster_msg;
    uint8_t serial_data[256] = {0};

    thruster_msg.data.push_back(27);
    pub.publish(thruster_msg);

    mSerial.Read(serial_data, 1);

    EXPECT_EQ(27, serial_data[0]);
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "serial_subscriber_test");
    ros::NodeHandle n;
    rs::SerialTB serial_tb;
    std::string UUT_port;

    rs::wait_for_param("thruster_serial_port");
    ros::param::get("thruster_serial_port", UUT_port);
    std::string testing_port = serial_tb.Start(UUT_port);
    mSerial.Open(testing_port.c_str(), B9600);

    pub = n.advertise<robosub::thruster>("thruster", 1);
    rs::wait_for_subscriber(pub);

    return RUN_ALL_TESTS();
}
