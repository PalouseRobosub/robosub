#include "robosub/Float32Stamped.h"

#include <Tca9545a.h>
#include <MS5837.h>
#include <ros.h>
#include <stdint.h>
#include <Wire.h>

/*
 * The MUX reset pin is on A1, which maps to 25 in the standard Arduino pinout.
 */
Tca9545a mux(25, false, false);

static constexpr unsigned int num_depth_sensors = 4;

MS5837 depth_sensors[num_depth_sensors];

/*
 * The depth sensor power control pin is A2, which maps to 26 in the standard
 * Arduino pinout.
 */
static constexpr int depth_power_control_pin = 26;

ros::NodeHandle n;

robosub::Float32Stamped depth_msg;
ros::Publisher depth_data_pub("depth", &depth_msg);

double cycle_delay = 0;
int temperature_update_period_s = 0;
float depth_offsets[num_depth_sensors];
ros::Time next_temperature_read;

void setup()
{
    /*
     * Turn on the depth sensor by writing high to the transistor gate.
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
     * Initialize the depth sensor with the fluid density of
     * water (997 kg/m^3) and initialize the I2C mux.
     */
    if(mux.init(Tca9545a::Channel::One))
    {
        n.logwarn("Failed to initialize the I2C mux.");
    }

    for (unsigned int i = 0; i < num_depth_sensors; ++i)
    {
        if (mux.setChannel(Tca9545a::toChannel(i + 1)))
        {
            n.logwarn("Failed to set MUX channel.");
        }

        if (depth_sensors[i].init())
        {
            n.logwarn("Failed to initialize the depth sensor.");
        }
        else
        {
            n.loginfo("Initialized depth sensor.");
        }
        depth_sensors[i].setFluidDensity(997.0f);
    }

    /*
     * Delay 500ms to ensure that the depth sensors have time to
     * properly initialize.
     */
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
     * Each conversion takes a minimum of 20ms, which means the rate can never
     * exceed 50Hz.
     */
    if (rate > 50)
    {
        rate = 50;
        n.logwarn("Depth rate was capped to 50Hz.");
    }

    /*
     * Load the depth sensor offset parameter from the parameter server.
     */
    for (unsigned int i = 0; i < num_depth_sensors; ++i)
    {
        char topic[] = "depth/offset/ ";
        topic[13] = static_cast<char>(i + '0');
        if (n.getParam(topic, &depth_offsets[i], 1) == false)
        {
            n.logwarn("Failed to load depth offset.");
            depth_offsets[i] = 0;
        }
    }

    /*
     * Load the temperature update period.
     */
    if (n.getParam("depth/temperature_update_period",
                &temperature_update_period_s, 1) == false)
    {
        n.logwarn("Failed to load temperature update period. "
                "Defaulting to 10 seconds.");
        temperature_update_period_s = 10;
    }

    cycle_delay = 1.0 / static_cast<float>(rate) * 1000;
    next_temperature_read = n.now();
}

void loop()
{
    if (n.now().toSec() > next_temperature_read.toSec())
    {
        for (unsigned int i = 0; i < num_depth_sensors; ++i)
        {
            if (depth_sensors[i].trigger_read(MS5837::Measurement::Pressure,
                        n.now()))
            {
                n.logwarn("Failed to trigger temperature read.");
            }
        }
        delay(20);
        for (unsigned int i = 0; i < num_depth_sensors; ++i)
        {
            if (depth_sensors[i].read(n.now()) != 1)
            {
                n.logwarn("Failed to read temperature conversion.");
            }
        }

        next_temperature_read = n.now();
        next_temperature_read += ros::Duration(temperature_update_period_s, 0);
    }

    /*
     * The depth sensor output specifies positive value as depth, however
     * the submarine prints depth as negative. Invert it and remove the
     * depth sensor offset.
     */
    for (unsigned int i = 0; i < num_depth_sensors; ++i)
    {
        if (depth_sensors[i].trigger_read(MS5837::Measurement::Pressure,
                    n.now()))
        {
            n.logwarn("Failed to trigger depth read.");
        }
    }
    delay(20);
    float total_depth = 0;
    for (unsigned int i = 0; i < num_depth_sensors; ++i)
    {
        switch (depth_sensors[i].read(n.now()))
        {
            case 0:
                n.logwarn("Read attempted before conversion completed.");
                break;
            case 1:
                n.logwarn("Failed to read depth conversion result.");
                break;
            default:
                total_depth += -1 * (depth_sensors[i].depth() +
                        depth_offsets[i]);
                break;
        }
    }

    depth_msg.header.stamp = n.now();
    depth_msg.data = total_depth/num_depth_sensors;
    depth_data_pub.publish(&depth_msg);
    n.spinOnce();
    delay(cycle_delay);
}
