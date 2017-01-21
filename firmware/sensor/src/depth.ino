/*
 * Author: Zachary Pratt
 *
 * Arduino using Adafruit unified sensor library to get data from the
 * BNO055 9-axis imu sensor and publish the data through ros as a quaternion message.
 *
 * Originally created to test effects of outside magnetic fields on
 * the orientation data out BNO055.
 */

#include <stdint.h>

#include <MS5837.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <Wire.h>

MS5837 ds;

ros::NodeHandle n;

std_msgs::Float32 depth_msg;

ros::Publisher depth_data_pub("depth", &depth_msg);

void setup() {
    n.initNode();
    n.advertise(depth_data_pub);
    n.loginfo("Starting setup.");
    delay(500);
    ds.init();
    ds.setFluidDensity(997.0f); // fluid density of freshwater
    delay(1000);
    n.loginfo("Setup Complete.");
}

void loop()
{
    n.loginfo("Reading Depth Sensor.");
    ds.read();
    depth_msg.data = ds.depth();
    depth_data_pub.publish(&depth_msg);

    n.loginfo("Spinning.");

    n.spinOnce();

    delay(100);
}
