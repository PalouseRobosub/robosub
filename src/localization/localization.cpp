#include "localization/localization_system.hpp"
#include "localization/robosub_sensors.h"

using namespace Eigen;

int main(int argc, char **argv)
{
    /* Initializes a new ROS node for the current process, and gives it the 
    name "localization". This name must be unique across the whole ROS system */
    ros::init(argc, argv, "localization");

    /* This call starts the ROS node that was created in the last step, and also
    provides access to the services that the ROS node provides, like advertising
    and subscribing to topics */
    ros::NodeHandle nh;

    /* Creates a publisher which advertises the "position/point" topic. This
    will be used to share our estimated position with the other ROS 
    subsystems */
    ros::Publisher loc_point_pub =
    nh.advertise<geometry_msgs::PointStamped>("position/point", 1);

    /* Set up pose publisher. This is necessary for visualizing in rviz. */
    ros::Publisher pose_pub =
        nh.advertise<geometry_msgs::PoseStamped>("rviz/cobalt/pose", 1);

    /* Wait for ros::Time to initialize. Ros::Time::isValid will return true
    if the time is non-zero. Helps with full-system synchronization. */
    while(!ros::Time::isValid())
    {
        usleep(10000);
    }

    /* Creates instances of the RobosubSensors and LocalizationSystem classes.
    RobosubSensors is designed to handle and sanitize input from outside topics,
    and the LocalizationSystem class manages the particle filter and kalman
    filter classes */
    RobosubSensors sensors;
    LocalizationSystem loc_system(nh, sensors);

    /* This provides system-wide access to a Remote Procedure Call (RPC) that
    will allow the filters to be reset to their initial configurations. This
    could be useful if the control system detects an error and requires a full
    system reset. */
    ros::ServiceServer reset_filter_service =
        nh.advertiseService("localization/reset",
                &LocalizationSystem::ResetFilterCallback, &loc_system);

    /* Create subscribers to the topics we want to consume to power the
    kalman filters and particle filters. These subscribers take 4 arguments. 
    First, a string with the topic name to subscribe to, then an int to define 
    the subscriber's queue size, next a function pointer for the function we 
    would like to call when new data arrives, and last a reference to the object
    who's method we should call. */

    //subscribe to depth sensor updates
    ros::Subscriber depth_sub = nh.subscribe("depth", 1,
            &RobosubSensors::InputDepth, &sensors);

    //subscribe to hydrophone pinger bearing updates
    ros::Subscriber hydrophones_bearing_sub =
        nh.subscribe("hydrophones/bearing", 1,
                &RobosubSensors::InputHydrophones, &sensors);

    //subscribe to linear acceleration updates from the IMU
    ros::Subscriber accel_sub = nh.subscribe("acceleration/linear", 1,
            &RobosubSensors::InputRelLinAcl, &sensors);

    //subscribe to orientation updates from the IMU
    ros::Subscriber orientation_sub = nh.subscribe("orientation", 1,
            &RobosubSensors::InputOrientation, &sensors);

    /* Setting the rate defines the shortest amount of time a single loop should
    take to run. This is used later via r.sleep() to make sure we run below this
    rate. There is no guarantee that we will reach the specified rate. */
    double rate;
    ROS_ERROR_COND(!ros::param::getCached("rate/localization",
            rate), "Failed to load localization rate.");
    ros::Rate r(rate);

    /* checks for node shutdown. When node is shutdown, ros::ok() will return
    false. This is required because the process running this code is tied to,
    but is not, the ROS Node itself. By making this check, our code will stop
    executing when the node is shutdown, either by ourselves or by an external
    command.*/
    while(ros::ok())
    {
        /* Calling ros::spinOnce fires all the callbacks that are pending from
        updates that have reached subscribers. These callbacks are ONLY handled
        when ros::spinOnce is called, or if ros::spin is called the callbacks
        will be handled until the node is shutdown. ros::spinOnce provides more
        control over the timing of the execution. */
        ros::spinOnce();

        /* ros::spinOnce handled the topic callbacks to the sensor class, and 
        the sensor class acts as a buffer for this incoming data. Now, the 
        update() function makes the kalman filter and particle filter consume 
        that buffered data, recalculate their internal states, and create a new
        position estimate. */
        loc_system.Update();

        /* these function calls retrieve data from the localization system that
        is already properly formatted for the message topics we want to publish.
        The publishers transparently handle making this data avaliable on their
        topics to other subsystems. */
        pose_pub.publish(loc_system.GetPoseMessage());
        loc_point_pub.publish(loc_system.GetLocalizationPoint());

        /* This consumes the rest of the time in our current cycle. */
        r.sleep();
    }

    return 0;
}
