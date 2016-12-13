#include "utility/test_tools.hpp"


namespace rs
{
void wait_for_param(const char *param_name, int timeout_seconds)
{
    ros::Time exit_time = ros::Time::now() + ros::Duration(timeout_seconds);
    while(ros::param::has(param_name) == false)
    {
        ROS_DEBUG_THROTTLE(1, "waiting for parameter: %s", param_name);
        if(ros::Time::now() < exit_time)
        {
            ros::WallDuration(0.1).sleep();
        }
        else //we timed out, time to die!
        {
            ROS_FATAL("timed out waiting for parameter: %s", param_name);
            exit(1);
        }
    }
}

void wait_for_subscriber(ros::Publisher pub, int timeout_seconds)
{
    ros::Time exit_time = ros::Time::now() + ros::Duration(timeout_seconds);
    while(pub.getNumSubscribers() == 0)
    {
        ROS_DEBUG_THROTTLE(1, "waiting for subscriber on: %s",
                           pub.getTopic().c_str());
        if(ros::Time::now() < exit_time)
        {
            ros::WallDuration(0.1).sleep();
        }
        else //we timed out, time to die!
        {
            ROS_FATAL("timed out waiting for subscriber: %s",
                      pub.getTopic().c_str());
            exit(1);
        }
    }
}
};
