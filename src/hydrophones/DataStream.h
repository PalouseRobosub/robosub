#ifndef DATASTREAM_H
#define DATASTREAM_H

#include "AnalogMeasurement.h"
#include "AnalogPacket.h"

#include <deque>
#include <vector>

/**
 * Represents a stream of analog samples from the hydrophones.
 */
class DataStream
{
    /**
     * Specifies the analog sample value that corresponds with a positive ping
     * start detection event.
     *
     * TODO: Select an appropriate threshold.
     */
    static constexpr uint16_t detection_threshold = 0;

    /**
     * Stores the period of the slowest analog waveform the stream represents
     * (25KHz).
     */
    static constexpr float period_s = 1.0 / 25000;

    /**
     * Stores the decay rate [0, 1] over a single period for the rectification
     * detector.
     */
    static constexpr float rectification_decay_per_cycle = 0.5;

    static_assert(rectification_decay_per_cycle <= 1 &&
                  rectification_decay_per_cycle >= 0,
                  "Decay rate must be a proportion.");

public:
    DataStream();

    void insert(vector<AnalogMeasurement> &new_measurements);

    void window(double start_time, double end_time);

    void clear();

    void get_measurements(vector<uint16_t> &samples, vector<float> &timestamps);

    bool get_ping_start_time(double &start_time);

private:
    /*
     * The stored data stream. Data is removed from the front of the deque as
     * necessary. If a ping event is detected, data will be continually stored
     * in the deque until the measurements are dumped to an external buffer
     * through a call to get_measurements.
     */
    std::deque<AnalogMeasurement> data;
};

#endif
