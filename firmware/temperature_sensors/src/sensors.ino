/**
 * @author Ryan Summers
 * @date 1-19-2018
 *
 * @brief This node measures the temperature of a probe that uses a resistor
 *        divider of a thermistor and a resistor.
 */

#include "robosub_msgs/Float32Stamped.h"
#include <ros.h>

/**
 * Describes the pin that the thermistor sensor is connected to.
 */
static constexpr int THERMISTOR_PIN = A0;

/**
 * The node handle for the node.
 */
ros::NodeHandle n;

/**
 * Publisher for the measured temperature (in Celcius) of the temperature
 * probe.
 */
robosub_msgs::Float32Stamped temp_msg;
ros::Publisher temp_data_pub("power/temp", &temp_msg);

/**
 * Publisher for the measured resistance of the sensor. This is useful to
 * detect the extremes of the temperature sensor's measurement capabilities.
 */
robosub_msgs::Float32Stamped resistance_msg;
ros::Publisher resistance_data_pub("power/resistance", &resistance_msg);

/**
 * Describes the chracteristics of the Thermistor using the Beta approximation.
 */
static constexpr float B = 3950;
static constexpr float Ro = 100000;
static constexpr float To = 25 + 273.15;

/**
 * Hardware characteristic that describes the resistance (in Ohms) of the lower
 * resistor on the resistor divider.
 */
static constexpr float R_LOWER = 100000;

/**
 * Oversampling rate on the analog temperature measurement.
 */
int oversample_rate = 25;

/**
 * Specifies the number of milliseconds to wait in between temperature
 * measurements.
 */
int cycle_delay = 0;

/**
 * Measures the temperature of a probe connected to a specified pin.
 *
 * @param _pin The pin that the temperature probe is connected to.
 *
 * @return The measured temperature (in Celcius).
 */
float get_temperature(const int _pin)
{
    /*
     * Read the ratio of Vcc that the sensor reads.
     */
    int adc_accumulator = 0;
    for (int i = 0; i < oversample_rate; ++i)
    {
        adc_accumulator += analogRead(_pin);
    }

    const int adc_measurement = adc_accumulator / oversample_rate;

    resistance_msg.header.stamp = n.now();
    resistance_msg.data =  R_LOWER * ((1023.0 / adc_measurement) -1);


    /*
     * Calculate temperature measurement of the thermistor using the Beta
     * approximation of the Steinhart-Hart equation.
     * https://en.wikipedia.org/wiki/Thermistor#B_or_%CE%B2_parameter_equation
     */
    const float inverse_temp = 1.0 / To +
            1.0 / B * log(1023.0 / adc_measurement - 1);

    const float temp_kelvin = 1.0 / inverse_temp;

    /*
     * Convert the temperature to celcius and return it.
     */
    return temp_kelvin - 273.15;
}

/**
 * Arduino setup function.
 *
 * @return None.
 */
void setup()
{
    n.initNode();
    n.advertise(temp_data_pub);
    n.advertise(resistance_data_pub);

    /*
     * Continually spin until the node is connected to the ROS system.
     */
    while (!n.connected())
    {
        n.spinOnce();
    }

    /*
     * Print out a message over the UART channel to indicate that the node
     * successfully started. Wait 300ms after the transmission to ensure that
     * the message is sent through UART hardware before returning from the
     * function.
     */
    n.loginfo("Temperature node connected.");
    delay(300);

    /*
     * Grab the oversampling rate and the node frequency.
     */
    if (!n.getParam("temperature/oversampling", &oversample_rate, 1))
    {
        n.logwarn("Failed to load temperature oversampling rate.");
        oversample_rate = 25;
    }

    /*
     * Grab the frequency that the node should run at.
     */
    int rate;
    if (!n.getParam("rate/temperature", &rate))
    {
        n.logwarn("Failed to load temperature rate. Defaulting to 10Hz.");
        rate = 10;
    }

    cycle_delay = 1000.0 / static_cast<float>(rate);
}

/**
 * Arduino loop function.
 *
 * @return None.
 */
void loop()
{
    const float temp_c = get_temperature(THERMISTOR_PIN);

    temp_msg.header.stamp = n.now();
    temp_msg.data = temp_c;

    temp_data_pub.publish(&temp_msg);
    resistance_data_pub.publish(&resistance_msg);

    n.spinOnce();
    delay(cycle_delay);
}
