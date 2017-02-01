#include "robosub/Float32ArrayStamped.h"
#include "tca9545a.h"

#include <MS5837.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <stdint.h>
#include <Wire.h>

/*
 * The MUX reset pin is on A1, which maps to 25 in the standard Arduino pinout.
 */
Tca9545a mux(25, false, false);

MS5837 depth_sensor[4];

ros::NodeHandle n;

robosub::Float32ArrayStamped depth_msg;

ros::Publisher depth_data_pub("depth", &depth_msg);

double cycle_delay = 0;
float depth_offset = 0;

void setup()
{
    n.initNode();
    n.advertise(depth_data_pub);

    /*
     * Delay to allow sensor to power up and ROS node to initialize.
     */
    while(n.connected() == false)
    {
        n.spinOnce();
    }

    /*
     * Display that the node connected and delay 300ms to ensure
     * that the message can be printed up the UART before
     * initializing the depth sensor.
     */
    n.loginfo("Node connected.");
    delay(300);

    /*
     * Initialize I2C to utilize a 100KHz clock instead of a
     * 400KHz clock. 400KHz has been shown to not work underwater
     * while 100KHz appears to function.
     */
    Wire.begin();
    Wire.setClock(100000);

    /*
     * Initialize the depth sensors with the fluid density of
     * water (997 kg/m^3).
     */
    mux.setChannel(Tca9545a::Channel::One);
    depth_sensor[0].init();
    depth_sensor[0].setFluidDensity(997.0f);
    mux.setChannel(Tca9545a::Channel::Two);
    depth_sensor[1].init();
    depth_sensor[1].setFluidDensity(997.0f);
    mux.setChannel(Tca9545a::Channel::Three);
    depth_sensor[2].init();
    depth_sensor[2].setFluidDensity(997.0f);
    mux.setChannel(Tca9545a::Channel::Four);
    depth_sensor[3].init();
    depth_sensor[3].setFluidDensity(997.0f);

    /*
     * Delay 500ms to ensure that the depth sensors have time to
     * properly initialize and then display a message to the
     * console that the depth sensors have initialized.
     */
    delay(500);
    n.loginfo("Depth sensor initialized.");

    /*
     * Once the node is initialized, grab the depth rate if it is
     * available. Otherwise, default to 10Hz and calculate the
     * delay in milliseconds between loops.
     */
    int rate;
    if (n.getParam("rate/depth", &rate, 1) == false)
    {
        rate = 10;
        n.logwarn("Failed to load depth rate. Defaulting to 10Hz.");
    }

    /*
     * The depth sensor has been empirically tested to only work
     * at relatively low update frequencies of 20Hz.
     */
    if (rate > 20)
    {
        rate = 20;
        n.logwarn("Depth rate was capped to 20Hz.");
    }

    /*
     * Load the depth sensor offset parameter from the parameter server.
     */
    if (n.getParam("depth/offset", &depth_offset, 1) == false)
    {
        n.logwarn("Failed to load depth offset.");
        depth_offset = 0;
    }

    cycle_delay = 1.0 / static_cast<float>(rate) * 1000;
}

void loop()
{
    float depths[4];

    mux.setChannel(Tca9545a::Channel::One);
    depth_sensor[0].read();
    mux.setChannel(Tca9545a::Channel::Two);
    depth_sensor[1].read();
    mux.setChannel(Tca9545a::Channel::Three);
    depth_sensor[2].read();
    mux.setChannel(Tca9545a::Channel::Four);
    depth_sensor[3].read();

    depth_msg.header.stamp = n.now();
    depth_msg.data = depths;
    depth_msg.data_length = 4;

    /*
     * The depth sensor output specifies positive value as depth,
     * however the submarine prints depth as negative. Invert it
     * and remove the depth sensor offset.
     */
    depths[0] = -1 * (depth_sensor[0].depth() + depth_offset);
    depths[1] = -1 * (depth_sensor[1].depth() + depth_offset);
    depths[2] = -1 * (depth_sensor[2].depth() + depth_offset);
    depths[3] = -1 * (depth_sensor[3].depth() + depth_offset);

    n.spinOnce();
    delay(cycle_delay);
}
