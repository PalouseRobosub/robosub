#include <gtest/gtest.h>
#include "ros/ros.h"
#include "robosub_msgs/control.h"
#include "utility/test_tools.hpp"
#include <vector>
#include <string>
#include "robosub_msgs/Float32Stamped.h"
#include "robosub_msgs/Euler.h"

ros::Publisher pub;


//function for extracting the depth data
double get_roll_data(const robosub_msgs::Euler::ConstPtr& msg)
{
    return msg->roll;
}
TEST(ControlSystem, roll)
{
    ROS_DEBUG("Starting roll test");
    double test_roll = 25;
    double overshoot_allowed = 3;
    double average_threshold = 2.0;
    double std_dev_allowed = 1.5;

    rs::SubscriberAnalyzer<robosub_msgs::Euler> analyzer;
    analyzer.Init("pretty/orientation", &get_roll_data);

    robosub_msgs::control msg;

    //keep the sub steady
    msg.forward_state = robosub_msgs::control::STATE_ERROR;
    msg.forward = 0;
    msg.strafe_state = robosub_msgs::control::STATE_ERROR;
    msg.strafe_left = 0;
    msg.yaw_state = robosub_msgs::control::STATE_RELATIVE;
    msg.yaw_left = 0;
    msg.dive_state = robosub_msgs::control::STATE_ABSOLUTE;
    msg.dive = -1;
    msg.pitch_state = robosub_msgs::control::STATE_ABSOLUTE;
    msg.pitch_down = 0;

    //just go to roll goal
    msg.roll_state = robosub_msgs::control::STATE_ABSOLUTE;
    msg.roll_right = test_roll;

    //fill out a control message to roll
    pub.publish(msg);
    analyzer.Start();

    ROS_INFO("rolling to test angle");
    //wait for the sub to reach its roll
    ros::Time exit_time = ros::Time::now() + ros::Duration(10);
    while (ros::Time::now() < exit_time)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    analyzer.Stop();

    //confirm we didn't roll too far
    EXPECT_LT(analyzer.GetMax(), test_roll + overshoot_allowed);
    //EXPECT_GT(analyzer.GetAverage(), test_roll - overshoot_allowed);

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
    ROS_INFO_STREAM("Roll Average: " << analyzer.GetAverage());
    ROS_INFO_STREAM("Roll STD Dev: " << analyzer.GetStandardDeviation());
    EXPECT_NEAR(test_roll, analyzer.GetAverage(), average_threshold);
    EXPECT_LT(analyzer.GetStandardDeviation(), std_dev_allowed);
}

double get_pitch_data(const robosub_msgs::Euler::ConstPtr& msg)
{
    return msg->pitch;
}
TEST(ControlSystem, pitch)
{
    ROS_DEBUG("Starting pitch test");
    double test_pitch = 15;
    double overshoot_allowed = 2.5;
    double average_threshold = 2.0;
    double std_dev_allowed = 1.5;

    rs::SubscriberAnalyzer<robosub_msgs::Euler> analyzer;
    analyzer.Init("pretty/orientation", &get_pitch_data);

    robosub_msgs::control msg;

    //keep the sub steady
    msg.forward_state = robosub_msgs::control::STATE_ERROR;
    msg.forward = 0;
    msg.strafe_state = robosub_msgs::control::STATE_ERROR;
    msg.strafe_left = 0;
    msg.yaw_state = robosub_msgs::control::STATE_RELATIVE;
    msg.yaw_left = 0;
    msg.dive_state = robosub_msgs::control::STATE_ABSOLUTE;
    msg.dive = -1;
    msg.roll_state = robosub_msgs::control::STATE_ABSOLUTE;
    msg.roll_right = 0;

    //just go to pitch goal
    msg.pitch_state = robosub_msgs::control::STATE_ABSOLUTE;
    msg.pitch_down = test_pitch;

    //fill out a control message to pitch
    pub.publish(msg);
    analyzer.Start();

    ROS_INFO("pitching to test angle");
    //wait for the sub to reach its pitch
    ros::Time exit_time = ros::Time::now() + ros::Duration(10);
    while (ros::Time::now() < exit_time)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    analyzer.Stop();

    //confirm we didn't pitch too far
    EXPECT_LT(analyzer.GetMax(), test_pitch + overshoot_allowed);
    //EXPECT_GT(analyzer.GetAverage(), test_pitch - overshoot_allowed);

    ROS_INFO("Maintaining pitch to check steady-state oscillation...");
    analyzer.ClearData();
    ROS_INFO("Start collecting data.");
    analyzer.Start();
    //wait for 10 seconds to measure wiggle
    exit_time = ros::Time::now() + ros::Duration(10);
    while (ros::Time::now() < exit_time)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    ROS_INFO("Stopping analysis");
    analyzer.Stop();

    //confirm pitch is stable
    ROS_INFO_STREAM("Pitch Average: " << analyzer.GetAverage());
    ROS_INFO_STREAM("Pitch STD Dev: " << analyzer.GetStandardDeviation());
    EXPECT_NEAR(test_pitch, analyzer.GetAverage(), average_threshold);
    EXPECT_LT(analyzer.GetStandardDeviation(), std_dev_allowed);
}

double get_yaw_data(const robosub_msgs::Euler::ConstPtr& msg)
{
    return fabs(msg->yaw);
}
TEST(ControlSystem, yaw)
{
    ROS_DEBUG("Starting yaw test");
    double test_yaw = 180;
    double overshoot_allowed = 18.0;
    double average_threshold = 4.0;
    double std_dev_allowed = 2.0;

    rs::SubscriberAnalyzer<robosub_msgs::Euler> analyzer;
    analyzer.Init("pretty/orientation", &get_yaw_data);

    robosub_msgs::control msg;

    //keep the sub steady
    msg.forward_state = robosub_msgs::control::STATE_ERROR;
    msg.forward = 0;
    msg.strafe_state = robosub_msgs::control::STATE_ERROR;
    msg.strafe_left = 0;
    msg.pitch_state = robosub_msgs::control::STATE_ABSOLUTE;
    msg.pitch_down = 0;
    msg.dive_state = robosub_msgs::control::STATE_ABSOLUTE;
    msg.dive = -1;
    msg.roll_state = robosub_msgs::control::STATE_ABSOLUTE;
    msg.roll_right = 0;

    //just go to yaw goal
    msg.yaw_state = robosub_msgs::control::STATE_ABSOLUTE;
    msg.yaw_left = test_yaw;

    //fill out a control message to yaw
    pub.publish(msg);
    analyzer.Start();

    ROS_INFO("yawing to test angle");
    //wait for the sub to reach its yaw
    ros::Time exit_time = ros::Time::now() + ros::Duration(20);
    while (ros::Time::now() < exit_time)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    analyzer.Stop();

    //confirm we didn't yaw too far
    EXPECT_LT(analyzer.GetMax(), test_yaw + overshoot_allowed);
    //EXPECT_GT(analyzer.GetAverage(), test_yaw - overshoot_allowed);

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
    ROS_INFO_STREAM("Yaw Average: " << analyzer.GetAverage());
    ROS_INFO_STREAM("Yaw STD Dev: " << analyzer.GetStandardDeviation());
    EXPECT_NEAR(test_yaw, analyzer.GetAverage(), average_threshold);
    EXPECT_LT(analyzer.GetStandardDeviation(), std_dev_allowed);
}

//function for extracting the depth data
double get_depth_data(const robosub_msgs::Float32Stamped::ConstPtr& msg)
{
    return msg->data;
}

TEST(ControlSystem, depth)
{
    ROS_DEBUG("Starting depth test");
    double test_depth = -3;
    double overshoot_allowed  = 0.3;
    double average_threshold = 0.1;
    double std_dev_allowed = 0.05;

    rs::SubscriberAnalyzer<robosub_msgs::Float32Stamped> analyzer;

    analyzer.Init("depth", &get_depth_data);

    robosub_msgs::control msg;
    //keep the sub steady
    msg.forward_state = robosub_msgs::control::STATE_ERROR;
    msg.forward = 0;
    msg.strafe_state = robosub_msgs::control::STATE_ERROR;
    msg.strafe_left = 0;
    msg.yaw_state = robosub_msgs::control::STATE_RELATIVE;
    msg.yaw_left = 0;
    msg.roll_state = robosub_msgs::control::STATE_ABSOLUTE;
    msg.roll_right = 0;
    msg.pitch_state = robosub_msgs::control::STATE_ABSOLUTE;
    msg.pitch_down = 0;

    //just go to depth
    msg.dive_state = robosub_msgs::control::STATE_ABSOLUTE;
    msg.dive = test_depth;

    //fill out a control message to stay level and go to depth
    pub.publish(msg);
    analyzer.Start();

    ROS_INFO("Diving to depth");
    //wait for the sub to reach its depth
    ros::Time exit_time = ros::Time::now() + ros::Duration(10);
    while (ros::Time::now() < exit_time)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    analyzer.Stop();

    //confirm we didn't dive too deep
    EXPECT_LT(test_depth - overshoot_allowed, analyzer.GetMin());
    //EXPECT_GT(test_depth + overshoot_allowed, analyzer.GetAverage());

    ROS_INFO("Maintaining depth to check steady-state oscillation...");
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
    ROS_INFO_STREAM("Depth Average: " << analyzer.GetAverage());
    ROS_INFO_STREAM("Depth STD Dev: " << analyzer.GetStandardDeviation());
    EXPECT_NEAR(test_depth, analyzer.GetAverage(), average_threshold);
    EXPECT_LT(analyzer.GetStandardDeviation(), std_dev_allowed);
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "control_system_test");
    ros::NodeHandle n;

    pub = n.advertise<robosub_msgs::control>("control", 1);

    ROS_INFO("Waiting for control subscriber...");
    rs::wait_for_subscriber(pub, -1);

    robosub_msgs::control msg;

    //Dive to initial depth
    msg.forward_state = robosub_msgs::control::STATE_ERROR;
    msg.forward = 0;
    msg.strafe_state = robosub_msgs::control::STATE_ERROR;
    msg.strafe_left = 0;
    msg.yaw_state = robosub_msgs::control::STATE_RELATIVE;
    msg.yaw_left = 0;
    msg.dive_state = robosub_msgs::control::STATE_ABSOLUTE;
    msg.dive = -1;
    msg.pitch_state = robosub_msgs::control::STATE_ABSOLUTE;
    msg.pitch_down = 0;
    msg.roll_state = robosub_msgs::control::STATE_ABSOLUTE;
    msg.roll_right = 0;

    //fill out a control message to dive
    pub.publish(msg);

    //wait until at depth
    ros::Time exit_time = ros::Time::now() + ros::Duration(10);
    while (ros::Time::now() < exit_time)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    ROS_INFO("Took 10 seconds to get to depth. Starting tests.");

    return RUN_ALL_TESTS();
}
