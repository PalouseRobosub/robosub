#include <gtest/gtest.h>
#include "ros/ros.h"
#include "robosub/control.h"
#include "utility/test_tools.hpp"
#include <vector>
#include <string>
#include "robosub/Float32Stamped.h"
#include "robosub/Euler.h"

ros::Publisher pub;


//function for extracting the depth data
double get_roll_data(const robosub::Euler::ConstPtr& msg)
{
    return msg->roll;
}
TEST(ControlSystem, roll)
{
    double test_roll = 25;
    double overshoot_allowed = 1;
    double average_threshold = 0.5;
    double std_dev_allowed = 0.9;

    rs::SubscriberAnalyzer<robosub::Euler> analyzer;
    analyzer.Init("pretty/orientation", &get_roll_data);

    robosub::control msg;

    //keep the sub steady
    msg.forward_state = robosub::control::STATE_ERROR;
    msg.forward = 0;
    msg.strafe_state = robosub::control::STATE_ERROR;
    msg.strafe_left = 0;
    msg.yaw_state = robosub::control::STATE_RELATIVE;
    msg.yaw_left = 0;
    msg.dive_state = robosub::control::STATE_ABSOLUTE;
    msg.dive = -1;
    msg.pitch_state = robosub::control::STATE_ABSOLUTE;
    msg.pitch_down = 0;

    //just go to roll goal
    msg.roll_state = robosub::control::STATE_ABSOLUTE;
    msg.roll_right = test_roll;

    //fill out a control message to roll
    pub.publish(msg);
    analyzer.Start();

    ROS_INFO("rolling to test angle");
    //wait for 5 secosds for the sub to reach its roll
    ros::Time exit_time = ros::Time::now() + ros::Duration(10);
    while (ros::Time::now() < exit_time)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    analyzer.Stop();

    //confirm we didn't roll too far
    EXPECT_GT(test_roll + overshoot_allowed, analyzer.GetMax());

    ROS_INFO("maintaining roll to check steady-state oscillation...");
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

    //confirm roll is stable
    EXPECT_NEAR(test_roll, analyzer.GetAverage(), average_threshold);
    EXPECT_LT(analyzer.GetStandardDeviation(), std_dev_allowed);
}

double get_pitch_data(const robosub::Euler::ConstPtr& msg)
{
    return msg->pitch;
}
TEST(ControlSystem, pitch)
{
    double test_pitch = 10;
    double overshoot_allowed = 2;
    double average_threshold = 0.5;
    double std_dev_allowed = 0.9;

    rs::SubscriberAnalyzer<robosub::Euler> analyzer;
    analyzer.Init("pretty/orientation", &get_pitch_data);

    robosub::control msg;

    //keep the sub steady
    msg.forward_state = robosub::control::STATE_ERROR;
    msg.forward = 0;
    msg.strafe_state = robosub::control::STATE_ERROR;
    msg.strafe_left = 0;
    msg.yaw_state = robosub::control::STATE_RELATIVE;
    msg.yaw_left = 0;
    msg.dive_state = robosub::control::STATE_ABSOLUTE;
    msg.dive = -1;
    msg.roll_state = robosub::control::STATE_ABSOLUTE;
    msg.roll_right = 0;

    //just go to roll goal
    msg.pitch_state = robosub::control::STATE_ABSOLUTE;
    msg.pitch_down = test_pitch;

    //fill out a control message to roll
    pub.publish(msg);
    analyzer.Start();

    ROS_INFO("pitching to test angle");
    //wait for 5 secosds for the sub to reach its pitch
    ros::Time exit_time = ros::Time::now() + ros::Duration(10);
    while (ros::Time::now() < exit_time)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    analyzer.Stop();

    //confirm we didn't roll too far
    EXPECT_GT(test_pitch + overshoot_allowed, analyzer.GetMax());

    ROS_INFO("maintaining pitch to check steady-state oscillation...");
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

    //confirm pitch is stable
    EXPECT_NEAR(test_pitch, analyzer.GetAverage(), average_threshold);
    EXPECT_LT(analyzer.GetStandardDeviation(), std_dev_allowed);
}

double get_yaw_data(const robosub::Euler::ConstPtr& msg)
{
    return msg->yaw;
}
TEST(ControlSystem, yaw)
{
    double test_yaw = 50;
    double overshoot_allowed = 2;
    double average_threshold = 1.0;
    double std_dev_allowed = 1.25;

    rs::SubscriberAnalyzer<robosub::Euler> analyzer;
    analyzer.Init("pretty/orientation", &get_yaw_data);

    robosub::control msg;

    //keep the sub steady
    msg.forward_state = robosub::control::STATE_ERROR;
    msg.forward = 0;
    msg.strafe_state = robosub::control::STATE_ERROR;
    msg.strafe_left = 0;
    msg.pitch_state = robosub::control::STATE_ABSOLUTE;
    msg.pitch_down = 0;
    msg.dive_state = robosub::control::STATE_ABSOLUTE;
    msg.dive = -1;
    msg.roll_state = robosub::control::STATE_ABSOLUTE;
    msg.roll_right = 0;

    //just go to roll goal
    msg.yaw_state = robosub::control::STATE_ABSOLUTE;
    msg.yaw_left = test_yaw;

    //fill out a control message to roll
    pub.publish(msg);
    analyzer.Start();

    ROS_INFO("yawing to test angle");
    //wait for 5 secosds for the sub to reach its yaw
    ros::Time exit_time = ros::Time::now() + ros::Duration(10);
    while (ros::Time::now() < exit_time)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    analyzer.Stop();

    //confirm we didn't yaw too far
    EXPECT_GT(test_yaw + overshoot_allowed, analyzer.GetMax());

    ROS_INFO("maintaining yaw to check steady-state oscillation...");
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

    //confirm yaw is stable
    EXPECT_NEAR(test_yaw, analyzer.GetAverage(), average_threshold);
    EXPECT_LT(analyzer.GetStandardDeviation(), std_dev_allowed);
}

//function for extracting the depth data
double get_depth_data(const robosub::Float32Stamped::ConstPtr& msg)
{
    return msg->data;
}

TEST(ControlSystem, depth)
{
    double test_depth = -2;
    double overshoot_allowed  = 0.1;
    double average_threshold = 0.05;
    double std_dev_allowed = 0.01;

    rs::SubscriberAnalyzer<robosub::Float32Stamped> analyzer;

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
    EXPECT_NEAR(test_depth, analyzer.GetAverage(), average_threshold);
    EXPECT_LT(analyzer.GetStandardDeviation(), std_dev_allowed);
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
