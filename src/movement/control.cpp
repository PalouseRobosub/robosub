#include "control_system.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <utility/ThrottledPublisher.hpp>

using namespace robosub;

/*
 * The control system object.
 */
ControlSystem *control_system;

/**
 * Handles incoming depth messages.
 *
 * @param depth_msg A pointer to a depth message that was received.
 *
 * @return None.
 */
void depthCallback(const std_msgs::Float32::ConstPtr& depth_msg)
{
    robosub::depth_stamped stamped;
    stamped.depth = depth_msg->data;
    control_system->InputDepthMessage(stamped);
}

/**
 * Main entry point.
 *
 * @param argc, argv Arguments provided to ROS.
 *
 * @return Zero upon completion.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "control");

    ros::NodeHandle nh;

    control_system = new ControlSystem();

    ros::Subscriber depth_sub = nh.subscribe("depth", 1, depthCallback);

    ros::Subscriber orientation_sub =
            nh.subscribe("orientation", 1, &ControlSystem::InputOrientationMessage, control_system);

    ros::Subscriber control_sub =
            nh.subscribe("control", 1, &ControlSystem::InputControlMessage, control_system);

    ros::Publisher pub = nh.advertise<robosub::thruster>("thruster", 1);

    rs::ThrottledPublisher<robosub::control_status>
            control_state_pub(std::string("control_status"), 1, 5);

    int rate;
    nh.getParam("control/rate", rate);
    ros::Rate r(rate);

    while(ros::ok())
    {
        control_state_pub.publish(control_system->GetControlStatus());
        pub.publish(control_system->CalculateThrusterMessage());
        ros::spinOnce();
        r.sleep();
    }

    delete control_system;

    return 0;
}
