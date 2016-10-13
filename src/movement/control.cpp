#include "control_system.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

ControlSystem *control_system;

void controlCallback(const robosub::control::ConstPtr& msg)
{
    control_system->InputControlMessage(*msg);
    //control_system->PublishThrusterMessage();
    //ROS_DEBUG("got control message");

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
  	ros::Publisher control_state_pub = nh.advertise<robosub::control>("current_control_state", 1);

    control_system = new ControlSystem(&nh, &pub);

    int rate;
    nh.getParam("control/rate", rate);
    ros::Rate r(rate);

    int i = 10;
    while(ros::ok())
    {
        if (i-- < 0)
        {
            control_state_pub.publish(control_system->PublishControlState());
            i = 10;
        }
        control_system->CalculateThrusterMessage();
        control_system->PublishThrusterMessage();
        ros::spinOnce();
        r.sleep();
    }

    delete control_system;

    return 0;
}
