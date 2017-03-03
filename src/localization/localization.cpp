#include "localization_system.hpp"

#include "robosub_sensors.h"

using namespace Eigen;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization");

    ros::NodeHandle nh;

    RobosubSensors sensors;

    // TODO: Change to use localization messages
    ros::Publisher loc_pub =
        nh.advertise<geometry_msgs::Vector3Stamped>("position", 1);

    ros::Publisher pose_pub =
        nh.advertise<geometry_msgs::PoseStamped>("cobalt/pose", 1);

    double num_particles;
    ros::param::getCached("localization/num_particles", num_particles);

    while(!ros::Time::isValid())
    {
        usleep(10000);
    }

    LocalizationSystem loc_system(&nh, &sensors, num_particles);

    // Service for resetting position and velocity
    ros::ServiceServer reset_filter_service =
        nh.advertiseService("reset_particle_filter",
                            &LocalizationSystem::ResetFilterCallback, &loc_system);

    ros::Subscriber depth_sub = nh.subscribe("depth", 1,
                                &RobosubSensors::InputDepth, &sensors);
    ros::Subscriber hydrophones_position_sub =
        nh.subscribe("hydrophones/position", 1,
                     &RobosubSensors::InputHydrophones, &sensors);
    ros::Subscriber accel_sub = nh.subscribe("acceleration/linear", 1,
                                &RobosubSensors::InputRelLinAcl, &sensors);
    ros::Subscriber orientation_sub = nh.subscribe("orientation", 1,
                                      &RobosubSensors::InputOrientation, &sensors);

    double rate;
    ros::param::getCached("localization/rate", rate);
    ros::Rate r(rate);

    ros::Time start_time;
    start_time = ros::Time::now();

    while(ros::ok())
    {
        ros::spinOnce();

        // Delay start by a few seconds so that sensor data can be all inputted
        ros::Duration delay_start = (ros::Time::now() - start_time);
        if(delay_start.toSec() > 1.0)
        {
            loc_system.Update();

            loc_pub.publish(loc_system.GetLocalizationMessage());
            pose_pub.publish(loc_system.GetPoseMessage());

        }

        r.sleep();
    }

    return 0;
}
