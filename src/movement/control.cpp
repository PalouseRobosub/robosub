#include "control_system.cpp"

ControlSystem *control_system;

void controlCallback(const robosub::control::ConstPtr& msg)
{
    control_system->InputControlMessage(*msg);
}

void orientationCallback(const geometry_msgs::Quaternion::ConstPtr& msg)
{
    control_system->InputOrientationMessage(*msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "control");

	ros::NodeHandle nh("control");

	ros::Subscriber controlsub = nh.subscribe("control", 1, controlCallback);
	ros::Subscriber orientationsub = nh.subscribe("orientation", 1, orientationCallback);
  	ros::Publisher pub = nh.advertise<robosub::thruster>("thruster", 1);

    control_system = new ControlSystem(&nh, &pub);

    ros::spin();

    delete control_system;

    return 0;
}
