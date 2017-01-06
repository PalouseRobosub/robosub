#include "localization_system.hpp"
#include <eigen3/Eigen/Dense>

using namespace Eigen;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization");

    ros::NodeHandle n;

    double num_particles;
    ros::param::getCached("localization/num_particles", num_particles);

    double rate;
    ros::param::getCached("localization/rate", rate);

    LocalizationSystem loc_system(num_particles);

    // TODO: Change to use localization messages
    ros::Publisher loc_pub = n.advertise<geometry_msgs::Vector3Stamped>("pf_position", 1);

    // Service for resetting position and velocity
    ros::ServiceServer reset_filter_service = n.advertiseService("reset_particle_filter", &LocalizationSystem::resetFilterCallback, &loc_system);

    ros::Subscriber depth_sub = n.subscribe("depth", 1, &LocalizationSystem::depthCallback, &loc_system);
    ros::Subscriber hydrophones_position_sub = n.subscribe("hydrophones/position", 1, &LocalizationSystem::hydrophoneCallback, &loc_system);
    ros::Subscriber accel_sub = n.subscribe("rs_lin_accel_data", 1, &LocalizationSystem::linAccelCallback, &loc_system);
    ros::Subscriber orientation_sub = n.subscribe("orientation", 1, &LocalizationSystem::orientationCallback, &loc_system);

    ros::Rate r(rate);

    while(ros::ok())
    {
        ros::spinOnce();

        loc_system.Update();

        geometry_msgs::Vector3Stamped pos = loc_system.GetLocalizationMessage();
        loc_pub.publish(pos);

        r.sleep();
    }

    return 0;
}
