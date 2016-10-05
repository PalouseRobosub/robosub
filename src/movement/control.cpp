#include "control_system.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

ControlSystem *control_system;

void controlCallback(const robosub::control::ConstPtr& msg)
{
    control_system->InputControlMessage(*msg);
    control_system->PublishThrusterMessage();
}

void sensorCallback(const geometry_msgs::QuaternionStamped::ConstPtr& quat_msg, const robosub::depth_stamped::ConstPtr& depth_msg)
{
    std::cout << depth_msg << std::endl;
    control_system->InputSensorMessages(*quat_msg, *depth_msg);
    control_system->PublishThrusterMessage();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "control");

	ros::NodeHandle nh("control");

    message_filters::Subscriber<geometry_msgs::QuaternionStamped> quat_sub(nh, "orientation", 1);
    message_filters::Subscriber<robosub::depth_stamped> string_sub(nh, "depth", 1);
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::QuaternionStamped, robosub::depth_stamped> sensor_sync_policy;
    message_filters::Synchronizer<sensor_sync_policy> sensor_sync(sensor_sync_policy(10), quat_sub, string_sub);
    sensor_sync.registerCallback(boost::bind(&sensorCallback, _1, _2));

	ros::Subscriber controlsub = nh.subscribe("control", 1, controlCallback);
	//ros::Subscriber orientationsub = nh.subscribe("orientation", 1, orientationCallback);

  	ros::Publisher pub = nh.advertise<robosub::thruster>("thruster", 1);

    control_system = new ControlSystem(&nh, &pub);

    ros::spin();

    delete control_system;

    return 0;
}
