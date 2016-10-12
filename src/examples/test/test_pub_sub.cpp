//this file is designed to test the pub_sub_example.cpp file, which subscribes
//to the degrees topic and publishes to the radians topic. To test, we're going
//to publish on the degrees topic and listen on the radians topic to confirm
//correct results
#include <gtest/gtest.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "utility/test_tools.hpp"


//insert tests here
TEST(PubSub, basicTest)
{
    //get a subscriber test bench to help with receiving messages
    rs::SubscriberTB<std_msgs::Float64> st;
    st.Init("radians");

    //get a nodehandle we can use to create a publisher
    ros::NodeHandle n;

    //create a publisher to send data
    ros::Publisher pub = n.advertise<std_msgs::Float64>("degrees", 1);

    //create an outgoing message
    std_msgs::Float64 out_msg;


    //wait for the UUT to boot up
    rs::wait_for_subscriber(pub, 1);

    //in this loop we send a few test messages and inspect the outputs of
    //the UUT
    for (int i=0; i < 5; i++)
    {
        //construct and publish an outgoing message
        out_msg.data = i;
        pub.publish(out_msg);

        //sleep a bit to give time for the message to get through and the UUT
        //to respond
        ros::WallDuration(0.1).sleep();

        //spinonce to allow messages to get received
        ros::spinOnce();

        //calculate the expected result
        double expected_result = i * 3.14/180;

        //use our subscriber testbench to grab the latest message received
        double received_result = st.GetLatestMsg().data;

        //test if the expected result is the same as the received result
        EXPECT_FLOAT_EQ(expected_result, received_result);
    }
}

int main(int argc, char *argv[])
{
    //initialize the google testing framework
    testing::InitGoogleTest(&argc, argv);

    //initialize our testbench node
    ros::init(argc, argv, "test_pub_sub");
    ros::NodeHandle nh;

    //run all of the tests above
    return RUN_ALL_TESTS();
}
