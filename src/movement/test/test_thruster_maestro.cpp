//This file is designed to test the Maestro Thruster node, which
//receives messages of the robosub/thruster type, parses the normalized value,
//and transmits the thruster signal over the serial port.
//This example sends a single thruster message with several thrusters inside,
//and then listens on a serial port to confirm that the received values are
//correct.
#include <gtest/gtest.h>
#include "ros/ros.h"
#include "robosub/thruster.h"
#include "utility/serial.hpp"
#include "utility/test_tools.hpp"
#include <vector>
#include <string>

rs::Serial mSerial;
ros::Publisher pub;
double byte_to_force_ratio(uint8_t byte2, uint8_t byte3);
std::vector<uint8_t> channels;
double max_thrust;

//The test will act like it is both the control system and the maestro
//Several values that the control system might send are sent to the thruster
//node. The results are then read from the port created by socat and compared
//to the expected values
TEST(Maestro, basicTest)
{
    //allocate an array of bytes that will be used to receive serial data from
    //the serial port. I use the arbitrarily large size of 256.
    uint8_t maestro_data[256] = {0};

    //create an empty thruster message to send to the thruster module
    robosub::thruster maestro_msg;
    for (uint8_t i = 0; i < channels.size(); i++)
    {
        maestro_msg.data.push_back(0.0);
    }

    //send initial message to make the thruster module send its reset signal.
    //This test isn't checking for it, so we just flush the serial buffer and
    //throw it away
    pub.publish(maestro_msg);
    ros::Duration(2).sleep();
    mSerial.Flush();

    //fill out data in the thruster message.
    maestro_msg.data.clear();
    maestro_msg.data.push_back(1.0);
    maestro_msg.data.push_back(0.7);
    maestro_msg.data.push_back(0.8);
    maestro_msg.data.push_back(-0.5);
    maestro_msg.data.push_back(-0.8);
    maestro_msg.data.push_back(-1.0);

    //publish the thruster message
    pub.publish(maestro_msg);
    ros::Duration(1).sleep();

    //after we published the thruster message, the thruster module should have
    //received the message and then sent some data down the serial port. Here
    //we read that data and verify it is correct
    for(unsigned int i = 0; i < maestro_msg.data.size(); ++i)
    {
        //read one thruster packet from the serial port
        mSerial.Read(maestro_data, 4);

        //calculate the thruster speed value of the packet
        double actual_force = byte_to_force_ratio(maestro_data[2], maestro_data[3]);

        //check to make sure the command and channel bytes are correct.
        EXPECT_EQ(maestro_data[0], 0x84);
        EXPECT_EQ(maestro_data[1], channels[i]);

        //compare and make sure the thruster speed value received is equal to
        //the value sent (include thruster software hard-limit throttling to
        //10 Amps).
        double expected_force = maestro_msg.data[i] * max_thrust;
        if (expected_force > 31.32)
        {
            expected_force = 31.32;
        }
        else if (expected_force < -24.78)
        {
            expected_force = -24.78;
        }

        //Since there are polynomial approximations involved, ensure that the
        //difference between expected and actual is within range. Allow up to
        //5% difference in values (2 Newtons).
        EXPECT_NEAR(actual_force, expected_force, 2);
    }
}

double byte_to_force_ratio(uint8_t byte2, uint8_t byte3)
{
  const uint16_t pulse_width = (byte3 << 7) | byte2;

  /*
   * Polynomial curve fitting data was calculated by fitting a
   * cubic polynomial to the x and y portions of the bluerobotics
   * T200 thruster pwm vs Thrust curves.
   */
  double force_kgf = 0;
  if (pulse_width < 1475)
  {
      force_kgf = pow(pulse_width, 3) * -4.85606515e-8 +
                  pow(pulse_width, 2) * 1.78638023e-4 +
                  pow(pulse_width, 2) * -0.20563003 +
                  70.4911410;
  }
  else if (pulse_width > 1525)
  {
      force_kgf = pow(pulse_width, 3) * -5.21026922e-8 +
                  pow(pulse_width, 2) * 2.80234148e-4 +
                  pow(pulse_width, 2) * -0.486381952 +
                  274.867233;
  }

  /*
   * Convert KgF to N and then divide by the maximum force.
   */
  return force_kgf * 9.80665 / max_thrust;
}

int main(int argc, char *argv[])
{
    //initialize the google testing framework
    testing::InitGoogleTest(&argc, argv);

    //initialize our ros node
    ros::init(argc, argv, "thruster_maestro_test");
    ros::NodeHandle n;

    XmlRpc::XmlRpcValue my_list;
    if (!ros::param::get("thrusters/mapping", my_list))
    {
        ROS_ERROR("Failed to load thruster mapping.");
        return -1;
    }

    if (!ros::param::get("thrusters/max_thrust", max_thrust))
    {
        ROS_ERROR("Failed to load maximum thrust.");
        return -1;
    }

    for (int i = 0; i < my_list.size(); ++i)
    {
        channels.push_back(static_cast<int>(my_list[i]["channel"]));
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

    //wait for the UUT to finish launching, can't run tests until the UUT has
    //set itself up. This function will wait for 3 seconds for the UUT to
    //initialize, then it will error out and fail the test
    rs::wait_for_subscriber(pub, 5);

    return RUN_ALL_TESTS();
}
