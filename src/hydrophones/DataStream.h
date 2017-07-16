#ifndef DATASTREAM_H
#define DATASTREAM_H

#include "AnalogPacket.h"
#include "AnalogMeasurement.h"
#include "rs_types.h"
#include "robosub/HydrophoneSamples.h"

#include <deque>

/**
 * Specifies the analog sample value that corresponds with a positive ping
 * start detection event.
 *
 * TODO: Select an appropriate threshold.
 */
static constexpr uint16_t detection_threshold = 0;

/**
 * Represents a stream of analog samples from the hydrophones.
 */
class DataStream
{
    /**
     * Stores information about a sequence of samples obtained from the
     * AnalogPacket for use in storing data into a deque.
     */
    struct ChannelPacket
    {
        /*
         * The sequence number of the data.
         */
        uint32_t sequence;

        /*
         * The measured analog samples.
         */
        AnalogMeasurement samples[AnalogPacket::samples_per_packet];
    };

    /**
     * Specifies the length of a hydrophone ping in microseconds.
     */
    static constexpr uint32_t ping_length_us = 4000;

public:
    DataStream(const uint8_t channel);

    result_t insert(AnalogPacket &packet);

    result_t get_measurements(robosub::HydrophoneSamples &packet);

    bool has_ping();

private:
    /*
     * Specifies the channel number of the packet that the stream relates to.
     */
    const uint8_t channel_number;

    /*
     * The stored data stream. Data is removed from the front of the deque as
     * necessary. If a ping event is detected, data will be continually stored
     * in the deque until the measurements are dumped to an external buffer
     * through a call to get_measurements.
     */
    std::deque<ChannelPacket> data;

    /*
     * Specified true if the start of a ping has been detected in the data
     * stream.
     */
    bool ping_start_detected;

    /*
     * The timestamp of the packetized stream after which the ping should be
     * completed.  This timestamp is used for determining when a stream
     * contains a full ping sample.
     */
    uint64_t ping_done_timestamp;
};

#endif
