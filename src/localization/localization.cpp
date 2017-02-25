#include "localization_system.hpp"
#include <eigen3/Eigen/Dense>
#include "filter_sensors.h"

using namespace Eigen;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization");

    ros::NodeHandle nh;

    FilterSensors sensors;

    // TODO: Change to use localization messages
    ros::Publisher loc_pub =
        nh.advertise<geometry_msgs::Vector3Stamped>("position", 1);

    double num_particles;
    ros::param::getCached("localization/num_particles", num_particles);

    LocalizationSystem loc_system(&sensors, nh, num_particles);

    // Service for resetting position and velocity
    ros::ServiceServer reset_filter_service =
        nh.advertiseService("reset_particle_filter",
                        &LocalizationSystem::resetFilterCallback, &loc_system);

    ros::Subscriber depth_sub = nh.subscribe("depth", 1,
                              &FilterSensors::InputDepth, &sensors);
    ros::Subscriber hydrophones_position_sub =
                         nh.subscribe("hydrophones/position", 1,
                         &FilterSensors::InputHydrophones, &sensors);
    ros::Subscriber accel_sub = nh.subscribe("acceleration/linear", 1,
                           &FilterSensors::InputRelLinAcl, &sensors);
    ros::Subscriber orientation_sub = nh.subscribe("orientation", 1,
                        &FilterSensors::InputOrientation, &sensors);

    double rate;
    ros::param::getCached("localization/rate", rate);
    ros::Rate r(rate);

    while(ros::ok())
    {
        ros::spinOnce();

        loc_system.Update();

        geometry_msgs::Vector3Stamped pos =
                                          loc_system.GetLocalizationMessage();
        loc_pub.publish(pos);

        r.sleep();
    }

    return 0;
}
