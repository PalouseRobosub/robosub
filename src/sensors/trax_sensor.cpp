#include "sensors/PniTrax.h"
#include "ros/ros.h"
#include "robosub/Trax.h"

using namespace robosub;

PniTrax imu;
ros::Publisher trax_publisher;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sensor");

    ros::NodeHandle nh;

    string port_name;
    ROS_FATAL_COND(nh.getParam("ports/trax", port_name) == false,
            "Failed to load TRAX serial port.");

    ROS_FATAL_COND(imu.init(port_name), "Failed to initialize TRAX.");

    trax_publisher = nh.advertise<robosub::Trax>("trax", 1);

    float rate = 20;
    if (nh.getParam("rate/imu", rate) == false)
    {
        ROS_WARN_STREAM("Failed to load TRAX rate. Falling back to 20 Hz.");
        rate = 20;
    }

    ros::Rate r(rate);
    ROS_INFO("TRAX initialized into AHRS mode.");

    while (ros::ok())
    {
        float roll, pitch, yaw;
        bool calibrated;
        uint8_t yaw_accuracy;

        ROS_FATAL_COND(imu.getRPY(roll, pitch, yaw, yaw_accuracy, calibrated),
                "Failed to get roll, pitch, and yaw from the TRAX.");

        robosub::Trax trax_message;
        trax_message.roll = roll;
        trax_message.pitch = pitch;
        trax_message.yaw = yaw;
        trax_message.yaw_accuracy = yaw_accuracy;
        trax_message.calibrated = calibrated;

        trax_publisher.publish(trax_message);

        ros::spinOnce();
        r.sleep();
    }
}
