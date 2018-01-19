#include "robosub/Float32Stamped.h"
#include <ros.h>

#define THERMISTOR_PIN A0

ros::NodeHandle n;
robosub::Float32Stamped temp_msg;
robosub::Float32Stamped resistance_msg;
ros::Publisher temp_data_pub("power/temp", &temp_msg);
ros::Publisher resistance_data_pub("power/resistance", &resistance_msg);

static constexpr float B = 3950;
static constexpr float Ro = 100000;
static constexpr float To = 25 + 273.15;
static constexpr float R_LOWER = 100000;

static constexpr int oversample_rate = 25;

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
    const float inverse_temp = 1.0 / To + 1.0 / B * log(1023.0 / adc_measurement - 1);
    const float temp_kelvin = 1.0 / inverse_temp;

    /*
     * Convert the temperature to celcius and return it.
     */
    return temp_kelvin - 273.15;
}

void setup()
{
    n.initNode();
    n.advertise(temp_data_pub);
    n.advertise(resistance_data_pub);
    while (!n.connected())
    {
        n.spinOnce();
    }

    n.loginfo("Temperature node connected.");
    delay(300);
}

void loop()
{
    const float temp_c = get_temperature(THERMISTOR_PIN);

    temp_msg.header.stamp = n.now();
    temp_msg.data = temp_c;

    temp_data_pub.publish(&temp_msg);
    resistance_data_pub.publish(&resistance_msg);

    n.spinOnce();
    delay(100);

}
