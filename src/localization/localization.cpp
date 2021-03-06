#include "localization/localization_system.hpp"
#include "localization/robosub_sensors.h"

using namespace Eigen;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization");

    ros::NodeHandle nh;

    //set up a regular position publisher as a point
    ros::Publisher loc_point_pub =
    nh.advertise<geometry_msgs::PointStamped>("position/point", 1);

    // Set up pose publisher. This is necessary for visualizing in rviz.
    ros::Publisher pose_pub =
        nh.advertise<geometry_msgs::PoseStamped>("rviz/cobalt/pose", 1);

    // Wait for ros::Time to initialize and return good values.
    while(!ros::Time::isValid())
    {
        usleep(10000);
    }

    RobosubSensors sensors;
    LocalizationSystem loc_system(nh, sensors);

    ros::ServiceServer reset_filter_service =
        nh.advertiseService("localization/reset",
                &LocalizationSystem::ResetFilterCallback, &loc_system);

    // RobosubSensors class handles all sensor callbacks
    ros::Subscriber depth_sub = nh.subscribe("depth", 1,
            &RobosubSensors::InputDepth, &sensors);
    ros::Subscriber hydrophones_bearing_sub =
        nh.subscribe("hydrophones/bearing", 1,
                &RobosubSensors::InputHydrophones, &sensors);
    ros::Subscriber accel_sub = nh.subscribe("acceleration/linear", 1,
            &RobosubSensors::InputRelLinAcl, &sensors);
    ros::Subscriber orientation_sub = nh.subscribe("orientation", 1,
            &RobosubSensors::InputOrientation, &sensors);

    ros::Subscriber control_accel = nh.subscribe("control_acceleration", 1,
            &RobosubSensors::InputAccelMsg, &sensors);

    // Load main loop update rate
    double rate;
    ROS_ERROR_COND(!ros::param::getCached("rate/localization",
            rate), "Failed to load localization rate.");
    ros::Rate r(rate);

    //this loop calls localization_system to run the code
    while(ros::ok())
    {
        ros::spinOnce();

        // Run localization system update. This will input sensor data and run
        // the filters.
        loc_system.Update();

        // Finally, publish our data - both as a pose and the point itself.
        pose_pub.publish(loc_system.GetPoseMessage());
        loc_point_pub.publish(loc_system.GetLocalizationPoint());

        r.sleep();
    }

    return 0;
}
