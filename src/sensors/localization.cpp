#include "localization_system.hpp"

/*
msg/PositionsStamped.msg
std_msgs/Header header
float32[] distances
geometry_msgs/Vector3[] positions
*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization");

    ros::NodeHandle n;

    LocalizationSystem loc_system(1.0/33.0, 100);

    // TODO: Change to use localization messages
    ros::Publisher loc_pub = n.advertise<geometry_msgs::Vector3>("pf_position", 1);

    // Service for resetting position and velocity
    ros::ServiceServer reset_filter_service = n.advertiseService("reset_particle_filter", &LocalizationSystem::resetFilterCallback, &loc_system);

    ros::Subscriber depth_sub = n.subscribe("depth", 1, &LocalizationSystem::depthCallback, &loc_system);
    ros::Subscriber hydrophones_position_sub = n.subscribe("hydrophones/position", 1, &LocalizationSystem::hydrophoneCallback, &loc_system);

    ros::Rate r(40.0);

    while(ros::ok())
    {
        ros::spinOnce();
        loc_system.Update();

        //geometry_msgs::Vector3 pos = loc_system.GetLocalizationMessage();
        //loc_pub.publish(pos);

        r.sleep();
    }

    return 0;
}
