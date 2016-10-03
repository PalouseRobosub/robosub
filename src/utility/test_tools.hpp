#ifndef __SYNC_TOOLS_HPP__
#define __SYNC_TOOLS_HPP__

#include "ros/ros.h"

namespace rs
{
    void wait_for_param(const char *param_name);
    void wait_for_subscriber(ros::Publisher pub);
    void wait_for_publisher(ros::Subscriber pub);
};

#endif
