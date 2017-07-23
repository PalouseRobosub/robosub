#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <cstdint>

struct AnalogMeasurement
{
    typedef int16_t SampleType;

    /*
     * Specifies the time at which the sample was taken in seconds. This is a
     * relative time from the start of sampling.
     */
    double timestamp;

    /*
     * The actual analog sample measurement.
     */
    SampleType sample;
};

#endif
