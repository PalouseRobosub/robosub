/*
 * Author: Zachary Pratt
 *
 * @brief Arduino uno is used for reading the BlueRobotics MS5837 depth sensor.
 */
#include "Float32Stamped.h"


#include <MS5837.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <stdint.h>
#include <Wire.h>

MS5837 depth_sensor;

ros::NodeHandle n;

robosub::Float32Stamped depth_msg;

ros::Publisher depth_data_pub("depth", &depth_msg);

double cycle_delay = 0;

void setup()
{
    n.initNode();
    n.advertise(depth_data_pub);

    /*
     * Initialize the depth sensor with the fluid density of
     * water (997 kg/m^3).
     */
    depth_sensor.init();
    depth_sensor.setFluidDensity(997.0f);

    /*
     * Delay to allow sensor to power up and ROS node to initialize.
     */
    delay(500);
    while(n.connected() == false)
    {
        n.spinOnce();
    }

    /*
     * Once the node is initialized, grab the depth rate if it is
     * available. Otherwise, default to 10Hz and calculate the
     * delay in milliseconds between loops.
     */
    int rate;
    if (n.getParam("depth/rate", &rate, 1) == false)
    {
        rate = 10;
    }

    if (rate > 20)
    {
        rate = 20;
    }

    cycle_delay = 1 / static_cast<float>(rate) * 1000;
}

void loop()
{
    depth_sensor.read();
    depth_msg.header.stamp = n.now();
    depth_msg.data = depth_sensor.depth();
    depth_data_pub.publish(&depth_msg);

    n.spinOnce();
    delay(cycle_delay);
}
