#include "control_system.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <utility/ThrottledPublisher.hpp>

using namespace robosub;

ControlSystem *control_system;

void depthCallback(const std_msgs::Float32::ConstPtr& depth_msg)
{
    robosub::depth_stamped stamped;
    stamped.depth = depth_msg->data;
    control_system->InputDepthMessage(stamped);
}

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
            control_state_pub(std::string("current_control_state"), 1, 5);

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
