#include "sensors/PniTrax.h"
#include "ros/ros.h"
#include "robosub/Trax.h"
#include <std_srvs/Empty.h>
#include <string>

using namespace robosub;

PniTrax imu;

/*
 * Declaring trimming variables for correcting sensor mounting offsets.
 */

double last_roll = 0, last_pitch = 0, trim_roll = 0, trim_pitch = 0;

/**
 * ROS Service call for trimming sensor pitch and roll offsets.
 *
 * @param req The service request.
 * @param res The service response.
 *
 * @return True.
 */
bool trim(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Trimming TRAX sensor.");
    trim_roll = last_roll;
    trim_pitch = last_pitch;

    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sensor");

    ros::NodeHandle nh;

    /*
     * Initialize the TRAX IMU.
     */
    std::string port_name;
    ROS_FATAL_COND(nh.getParam("ports/trax", port_name) == false,
            "Failed to load TRAX serial port.");

    ROS_FATAL_COND(imu.init(port_name), "Failed to initialize TRAX.");

    /*
     * Set up the trim service and data publisher.
     */
    ros::ServiceServer trim_service = nh.advertiseService("trax/trim", trim);

    ros::Publisher trax_publisher = nh.advertise<robosub::Trax>("trax", 1);

    float rate = 20;
    if (nh.getParam("rate/imu", rate) == false)
    {
        ROS_WARN_STREAM("Failed to load TRAX rate. Falling back to 20 Hz.");
        rate = 20;
    }

    ros::Rate r(rate);
    ROS_INFO("TRAX initialized into AHRS mode.");

    /*
     * Begin the main ROS data loop.
     */
    while (ros::ok())
    {
        float roll, pitch, yaw;
        bool calibrated;
        uint8_t yaw_accuracy;

        ROS_FATAL_COND(imu.getRPY(roll, pitch, yaw, yaw_accuracy, calibrated),
                "Failed to get roll, pitch, and yaw from the TRAX.");

        /*
         * Update sensor trim variables.
         */
        last_pitch = pitch;
        last_roll = roll;

        /*
         * Construct the data message.
         */
        robosub::Trax trax_message;
        trax_message.roll = roll - trim_roll;
        trax_message.pitch = pitch - trim_pitch;
        trax_message.yaw = yaw;
        trax_message.yaw_accuracy = yaw_accuracy;
        trax_message.calibrated = calibrated;

        trax_publisher.publish(trax_message);

        ros::spinOnce();
        r.sleep();
    }
}
