#include "../../ros_lib/robosub/Float32ArrayStamped.h"
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

static constexpr int depth_power_control_pin = 26;
static constexpr int num_depth_sensors = 4;
MS5837 depth_sensor[num_depth_sensors];
float depths[num_depth_sensors];

Tca9545a::Channel channels[4] = {
    Tca9545a::Channel::One,
    Tca9545a::Channel::Two,
    Tca9545a::Channel::Three,
    Tca9545a::Channel::Four
};

ros::NodeHandle n;

robosub::Float32ArrayStamped depth_msg;

ros::Publisher depth_data_pub("depth", &depth_msg);

double cycle_delay = 0;
float depth_offset = 0;

void setup()
{
    /*
     * Turn on the depth sensors by writing high to the transistor gate.
     */
    pinMode(depth_power_control_pin, OUTPUT);
    digitalWrite(depth_power_control_pin, HIGH);

    /*
     * Delay to allow sensor to power up and ROS node to initialize.
     */
    n.initNode();
    n.advertise(depth_data_pub);
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
     * water (997 kg/m^3) and initialize the I2C mux.
     */
    if(mux.init(Tca9545a::Channel::One))
    {
        n.logwarn("Failed to initialize the I2C mux.");
    }

    for (int i = 0; i < num_depth_sensors; ++i)
    {
        if (mux.setChannel(channels[i]))
        {
            n.logwarn("Failed to set mux channel.");
        }

        if (depth_sensor[i].init())
        {
            n.logwarn("Failed to initialize the depth sensor.");
        }
        depth_sensor[i].setFluidDensity(997.0f);
        n.loginfo("Initialized depth sensor.");
    }

    /*
     * Delay 500ms to ensure that the depth sensors have time to
     * properly initialize.
     */
    n.loginfo("Depth sensors initialized.");
    delay(500);

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
    for (int i = 0; i < num_depth_sensors; ++i)
    {
        if (mux.setChannel(channels[i]))
        {
            n.logwarn("Failed to set MUX channel.");
        }
        if (depth_sensor[i].read())
        {
            n.logwarn("Failed to read depth sensor.");
        }

        /*
         * The depth sensor output specifies positive value as depth, however
         * the submarine prints depth as negative. Invert it and remove the
         * depth sensor offset.
         */
        depths[i] = -1 * (depth_sensor[i].depth() + depth_offset);
    }

    depth_msg.header.stamp = n.now();
    depth_msg.data = depths;
    depth_msg.data_length = num_depth_sensors;

    depth_data_pub.publish(&depth_msg);

    n.spinOnce();
    delay(cycle_delay);
}
