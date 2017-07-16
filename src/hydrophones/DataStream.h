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
     * Specifies the number of measurements to continually buffer.
     */
    static constexpr uint32_t measurements_to_buffer = 10000;

    /**
     * Specifies the length of a hydrophone ping in seconds.
     */
    static constexpr uint32_t ping_length_s = 0.004;

    /**
     * Specifies the analog sample value that corresponds with a positive ping
     * start detection event.
     *
     * TODO: Select an appropriate threshold.
     */
    static constexpr uint16_t detection_threshold = 0;

public:
    DataStream();

    void insert(vector<AnalogMeasurement> &new_measurements);

    void get_measurements(vector<uint16_t> &samples, vector<float> &timestamps);

    bool has_ping();

    float get_ping_start_timestamp();

private:
    /*
     * The stored data stream. Data is removed from the front of the deque as
     * necessary. If a ping event is detected, data will be continually stored
     * in the deque until the measurements are dumped to an external buffer
     * through a call to get_measurements.
     */
    std::deque<AnalogMeasurement> data;

    /*
     * Specified true if the start of a ping has been detected in the data
     * stream.
     */
    bool ping_start_detected;

    /*
     * The timestamp of the start of the ping.
     */
    float ping_start_timestamp;
};

#endif
