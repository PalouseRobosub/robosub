#include <gtest/gtest.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h"

//some global variables
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

//insert tests here
TEST(PubSub, basicTest)
{
    std_msgs::Float64 out_msg;
    SubscribeTester st;
    out_msg.data = 100;


    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("radians", 1, &SubscribeTester::Callback, &st);
    ros::Publisher pub = n.advertise<std_msgs::Float64>("degrees", 1);
    ros::WallDuration(1).sleep();
    pub.publish(out_msg);
    ros::WallDuration(0.5).sleep();
    ros::spinOnce();
    EXPECT_EQ(1, pub.getNumSubscribers());

    ROS_INFO("in: %lf, out: %lf", out_msg.data*3.14/180, st.GetMsg().data);
    EXPECT_FLOAT_EQ(out_msg.data*3.14/180, st.GetMsg().data);
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_pub_sub");
    ros::NodeHandle nh;

    return RUN_ALL_TESTS();
}
