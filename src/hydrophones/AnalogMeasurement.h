#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <cstdint>

struct AnalogMeasurement
{
    /*
     * Specifies the time at which the sample was taken in seconds. This is a
     * relative time from the start of sampling.
     */
    double timestamp;

    /*
     * The actual analog sample measurement.
     */
    uint16_t sample;
};

#endif
