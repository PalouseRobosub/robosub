#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "utility/serial.hpp"
#include <string>

rs::Serial mSerial;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_publisher");
    ros::NodeHandle n;

    ros::Publisher pub =
                      n.advertise<geometry_msgs::Quaternion>("orientation", 1);
    std::string port;

    if(!n.getParam("sensor_serial_port", port))
    {
      ROS_FATAL("no serial port specified, exiting!");
      exit(1);
    }

    mSerial.Open(port.c_str(), B9600);

    ros::Rate rate(10);

    while(ros::ok())
    {
      uint8_t serial_data[256];
      mSerial.Read(serial_data, 4);

      geometry_msgs::Quaternion my_orientation;
      my_orientation.x = serial_data[0];
      my_orientation.y = serial_data[1];
      my_orientation.z = serial_data[2];
      my_orientation.w = serial_data[3];

      pub.publish(my_orientation);
      rate.sleep();
    }
}
