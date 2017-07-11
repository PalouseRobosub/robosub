#ifndef MEASUREMENT_H
#define MEASUREMENT_H

struct AnalogMeasurement
{
    /*
     * Specifies the time at which the sample was taken (in microseconds)
     * (relative).
     */
    uint64_t timestamp;

    /*
     * The actual analog sample measurement.
     */
    uint16_t sample;
};

#endif
