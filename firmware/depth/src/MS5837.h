/*
Blue Robotics Arduino MS5837-30BA Pressure/Temperature Sensor Library
------------------------------------------------------------

Title: Blue Robotics Arduino MS5837-30BA Pressure/Temperature Sensor Library

Description: This library provides utilities to communicate with and to
read data from the Measurement Specialties MS5837-30BA pressure/temperature
sensor.

Authors: Rustom Jehangir, Blue Robotics Inc.
         Adam Šimko, Blue Robotics Inc.

-------------------------------
The MIT License (MIT)

Copyright (c) 2015 Blue Robotics Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-------------------------------
 */

#ifndef MS5837_H
#define MS5837_H

#include "Arduino.h"
#include <ros.h>

class MS5837
{
    static constexpr float Pa = 100.0f;
    static constexpr float bar = 0.001f;
    static constexpr float mbar = 1.0f;
    static constexpr int ms5837_address = 0x76;

    enum class Command : int
    {
        Reset = 0x1E,
        AdcRead = 0x00,
        PromRead = 0xA0,
        ConvertD1_8192 = 0x4A,
        ConvertD2_8192 = 0x5A
    };

public:
    enum class Measurement
    {
        Pressure,
        Temperature
    };

    MS5837();

    int init();

    /* Provide the density of the working fluid in kg/m^3. Default is for
     * seawater. Should be 997 for freshwater.
     */
    void setFluidDensity(float density);

    /* Triggers a read to occur on the sensor. The read may either be for
     * temperature or pressure. Once the read is triggered, 20ms are required
     * before a final result is available.
     */
    int trigger_read(Measurement sensor, ros::Time now);

    /* Reads the result of an ADC conversion if it is available.
     */
    int read(ros::Time now);

    /* This function loads the datasheet test case values to verify that
     * calculations are working correctly. No example checksum is provided
     * so the checksum test may fail.
     */
    void readTestCase();

    /* Pressure returned in mbar or mbar*conversion rate.
     */
    float pressure(float conversion = 1.0f);

    /* Temperature returned in deg C.
     */
    float temperature();

    /* Depth returned in meters (valid for operation in incompressible
     * liquids only. Uses density that is set for fresh or seawater.
     */
    float depth();

    /* Altitude returned in meters (valid for operation in air only).
     */
    float altitude();

private:
    uint16_t C[8];
    uint32_t D1, D2;
    int32_t TEMP;
    int32_t P;
    ros::Time conversion_ready;
    Measurement conversion;
    bool converting;

    float fluidDensity;

    /* Performs calculations per the sensor data sheet for conversion and
     * second order compensation.
     */
    void calculate();

    uint8_t crc4(uint16_t n_prom[]);
};

#endif  // MS5837_H
