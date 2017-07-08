#include "sensors/PniTrax.h"
#include "ros/ros.h"
#include "robosub/Euler.h"
#include <geometry_msgs/QuaternionStamped.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <string>
#include "tf/transform_datatypes.h"

using std::to_string;
using namespace robosub;

#define _PI_OVER_180 (3.14159) / 180.0

/*
 * Declare trimming variables for correcting sensor mounting offsets.
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

/**
 * Main program entry point.
 *
 * @return Zero upon success and non-zero upon error.
 */
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "trax");

    ros::NodeHandle nh("~");

    PniTrax imu;

    /*
     * Initialize the TRAX IMU.
     */
    std::string port_name;
    ROS_FATAL_COND(nh.getParam("port", port_name) == false,
                   "Failed to load TRAX serial port.");

    if (imu.init(port_name, PniTrax::Mode::AHRS) != 0)
    {
        ROS_FATAL("Failed to initialize TRAX.");
        return 1;
    }

    /*
     * Set up the trim service and data publisher.
     */
    ros::ServiceServer trim_service = nh.advertiseService("trim", trim);

    ros::Publisher trax_publisher =
              nh.advertise<geometry_msgs::QuaternionStamped>("orientation", 1);
    ros::Publisher trax_pretty_publisher =
                         nh.advertise<robosub::Euler>("pretty/orientation", 1);
    ros::Publisher trax_info_publisher =
                                     nh.advertise<std_msgs::String>("info", 1);

    float rate;
    if (nh.getParam("rate", rate) == false)
    {
        ROS_WARN("Failed to load TRAX rate. Falling back to 20 Hz.");
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
         * Transform the TRAX output values to conform with robosub standards
         * of roll_right, yaw_left, and pitch_down positives.
         */
        yaw = 180 - yaw;
        pitch *= -1;

        /*
         * Update sensor trim variables.
         */
        last_pitch = pitch;
        last_roll = roll;

        /*
         * Apply trim offsets
         */
        roll -= trim_roll;
        pitch -= trim_pitch;

        /*
         * Construct the data message.
         *
         * Yaw-right is positive on the TRAX. [0, 360)
         * Roll-left is positive on the TRAX. [-180, 180]
         * Pitch-down is positive on the TRAX. [-90, 90]
         */
        geometry_msgs::QuaternionStamped trax_message;
        trax_message.quaternion = tf::createQuaternionMsgFromRollPitchYaw(
                roll * _PI_OVER_180,
                pitch * _PI_OVER_180,
                yaw * _PI_OVER_180);
        trax_message.header.stamp = ros::Time::now();

        /*
         * Construct the human-readable message. Note that due to the trim,
         * these values may not lie within the TRAX specified ranges.
         */
        robosub::Euler trax_pretty_message;
        trax_pretty_message.roll = roll;
        trax_pretty_message.pitch = pitch;
        trax_pretty_message.yaw = yaw;

        std_msgs::String trax_status_message;
        trax_status_message.data = "Heading accuracy: " +
                to_string(yaw_accuracy) + "\nCalibrated: " +
                to_string(calibrated);

        /*
         * Publish orientation messages and status message, then continue.
         */
        trax_publisher.publish(trax_message);
        trax_pretty_publisher.publish(trax_pretty_message);
        trax_info_publisher.publish(trax_status_message);

        ros::spinOnce();
        r.sleep();
    }
}
