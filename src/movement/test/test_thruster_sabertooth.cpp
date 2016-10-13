#include <gtest/gtest.h>
#include "ros/ros.h"
#include "robosub/thruster.h"
#include "utility/serial_testbench.hpp"
#include "utility/serial.hpp"

class SubscribeTester
{
public:
    SubscribeTester()
    {}
    ~SubscribeTester()
    {}
    void Callback(const std_msgs::Float64::ConstPtr& msg)
    {
        m_msg = *msg;
    }
    std_msgs::Float64 GetMsg()
    {
        return m_msg;
    }

private:
    std_msgs::Float64 m_msg;

};


int main(int argc, char *argv[])
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "");
  ros::NodeHandle n;

  return RUN_ALL_TESTS;
}
