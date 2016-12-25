#include <gtest/gtest.h>
#include "ros/ros.h"
#include "robosub/control.h"
#include "utility/test_tools.hpp"
#include <vector>
#include <string>
#include "robosub/depth_stamped.h"

ros::Publisher pub;

//function for extracting the depth data
double get_depth_data(const robosub::depth_stamped::ConstPtr& msg)
{
    return msg->depth;
}

TEST(ControlSystem, depth)
{
    double test_depth = -2;
    double overshoot_allowed  = 0.1;

    rs::SubscriberAnalyzer<robosub::depth_stamped> analyzer;

    analyzer.Init("depth", &get_depth_data);

    robosub::control msg;
    //keep the sub steady
    msg.forward_state = robosub::control::STATE_ERROR;
    msg.forward = 0;
    msg.strafe_state = robosub::control::STATE_ERROR;
    msg.strafe_left = 0;
    msg.yaw_state = robosub::control::STATE_RELATIVE;
    msg.yaw_left = 0;
    msg.roll_state = robosub::control::STATE_ABSOLUTE;
    msg.roll_right = 0;
    msg.pitch_state = robosub::control::STATE_ABSOLUTE;
    msg.pitch_down = 0;

    //just go to depth
    msg.dive_state = robosub::control::STATE_ABSOLUTE;
    msg.dive = test_depth;

    //fill out a control message to stay level and go to depth
    pub.publish(msg);
    analyzer.Start();

    ROS_INFO("diving to depth");
    //wait for 5 secosds for the sub to reach its depth
    ros::Time exit_time = ros::Time::now() + ros::Duration(10);
    while (ros::Time::now() < exit_time)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    analyzer.Stop();

    //confirm we didn't dive too deep
    EXPECT_LT(test_depth - overshoot_allowed, analyzer.GetMin());

    ROS_INFO("maintaining depth to check steady-state oscillation...");
    analyzer.ClearData();
    analyzer.Start();
    //wait for 10 seconds to measure wiggle
    exit_time = ros::Time::now() + ros::Duration(10);
    while (ros::Time::now() < exit_time)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    analyzer.Stop();

    //confirm depth is stable
    EXPECT_NEAR(test_depth, analyzer.GetAverage(), 0.05);
    EXPECT_LT(analyzer.GetStandardDeviation(), 0.01);
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "control_system_test");
    ros::NodeHandle n;

    pub = n.advertise<robosub::control>("control", 1);

    rs::wait_for_subscriber(pub, 5);

    return RUN_ALL_TESTS();
}
