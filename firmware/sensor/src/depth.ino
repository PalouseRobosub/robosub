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
#include <ros/time.h>
#include <std_msgs/Float32.h>
#include "depth_stamped.h"
#include <Wire.h>

MS5837 ds;

ros::NodeHandle n;

robosub::depth_stamped depth_msg;

ros::Publisher depth_data_pub("depth", &depth_msg);

void setup() {
    n.initNode();
    n.advertise(depth_data_pub);
    delay(500);
    ds.init();
    ds.setFluidDensity(997.0f); // fluid density of freshwater
    delay(1000);
}

void loop()
{
    ds.read();
    depth_msg.header.stamp = n.now();
    depth_msg.depth = ds.depth();
    depth_data_pub.publish(&depth_msg);

    n.spinOnce();

    delay(100);
}
