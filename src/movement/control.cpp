#include "control_system.h"
#include "std_msgs/Float32.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <utility/ThrottledPublisher.hpp>

using namespace robosub;

ControlSystem *control_system;

void controlCallback(const robosub::control::ConstPtr& msg)
{
    control_system->InputControlMessage(*msg);
}

void orientationCallback(const geometry_msgs::Quaternion::ConstPtr& quat_msg)
{
    geometry_msgs::QuaternionStamped stamped;
    stamped.quaternion = *quat_msg;

    control_system->InputOrientationMessage(stamped);
}
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

    ros::Subscriber depthSub = nh.subscribe("depth", 1, depthCallback);
    ros::Subscriber orientationSub = nh.subscribe("orientation", 1, orientationCallback);

    ros::Subscriber controlsub = nh.subscribe("control", 1, controlCallback);

    ros::Publisher pub = nh.advertise<robosub::thruster>("thruster", 1);
    rs::ThrottledPublisher<robosub::control_status> control_state_pub(std::string("current_control_state"), 1, 5);

    control_system = new ControlSystem();

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

    return 0;
}
