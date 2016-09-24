#include "ros/ros.h"
#include "std_msgs/String.h"
ros::Publisher pub;



void printargv(char **argv)
{
    int i = 0;
    std::cout << "environment:" << std::endl;
    while(argv[i] != 0)
    {
        std::cout << argv[i] << std::endl;
        ++i;
    }
}
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
int main(int argc, char **argv, char **env)
{
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    //printargv(argv);
    //printargv(env);
    ros::init(argc, argv, "fork_child");
    ROS_INFO("new child node here!");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;
    ros::Rate rate(1);

    /**
     * The subscribe() call is how you tell ROS that you want to receive messages
     * on a given topic.  This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing.  Messages are passed to a callback function, here
     * called chatterCallback.  subscribe() returns a Subscriber object that you
     * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
     * object go out of scope, this callback will automatically be unsubscribed from
     * this topic.
     *
     * The second parameter to the subscribe() function is the size of the message
     * queue.  If messages are arriving faster than they are being processed, this
     * is the number of messages that will be buffered up before beginning to throw
     * away the oldest ones.
     */
    pub = n.advertise<std_msgs::String>("chatter", 1);

    /**
     * ros::spin() will enter a loop, pumping callbacks.  With this version, all
     * callbacks will be called from within this thread (the main one).  ros::spin()
     * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
     */
    while(ros::ok())
    {
        std_msgs::String msg;
        msg.data = "hello world";
        pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
