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

void orientationCallback(const geometry_msgs::QuaternionStamped::ConstPtr& quat_msg)
{
    control_system->InputOrientationMessage(*quat_msg);
}
void depthCallback(const robosub::depth_stamped::ConstPtr& depth_msg)
{
    control_system->InputDepthMessage(*depth_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "control");

	ros::NodeHandle nh;

    ros::Subscriber depthSub = nh.subscribe("depth", 1, depthCallback);
    ros::Subscriber orientationSub = nh.subscribe("orientation", 1, orientationCallback);

	ros::Subscriber controlsub = nh.subscribe("control", 1, controlCallback);
	//ros::Subscriber orientationsub = nh.subscribe("orientation", 1, orientationCallback);

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
            i = 0;
        }
        control_system->CalculateThrusterMessage();
        control_system->PublishThrusterMessage();
        ros::spinOnce();
        r.sleep();
    }

    delete control_system;

    return 0;
}
