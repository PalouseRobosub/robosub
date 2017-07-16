#include "DataStream.h"

#include "AnalogPacket.h"
#include "robosub/HydrophoneSamples.h"

#include <cstdint>
#include <vector>

using std::vector;

/**
 * Constructor.
 */
DataStream::DataStream() :
    ping_start_detected(false),
    ping_start_timestamp(0)
{
}

/**
 * Extract the data from internal buffering to a list of timestamp/value pairs
 * for the channel.
 *
 * @param samples The location to store the received analog samples.
 * @param timestampes The location to store the corresponding timestamps for
 *        each sample.
 *
 * @return None.
 */
void DataStream::get_measurements(vector<uint16_t> &samples,
                                 vector<float> &timestamps)
{
    /*
     * Convert all data from the deque into individual measurements and copy
     * them into the provided buffer.
     */
    timestamps.clear();
    samples.clear();

    for (size_t i = 0; i < data.size(); ++i)
    {
        samples.push_back(data[i].sample);
        timestamps.push_back(data[i].timestamp);
    }

    ping_start_timestamp = 0;
    ping_start_detected = false;

    /*
     * Remove the old data from buffering. Keep a small subsection of the most
     * recent past.
     */
    while (data.size() >= measurements_to_buffer)
    {
        data.pop_front();
    }
}

/**
 * Append data into the stream.
 *
 * @param measurements A vector of measurements to add to the stream.
 *
 * @return None.
 */
void DataStream::insert(vector<AnalogMeasurement> &measurements)
{
    for (size_t i = 0; i < measurements.size(); ++i)
    {
        /*
         * Check the new measurement for the detection threshold.
         */
        if (ping_start_detected == false &&
                measurements[i].sample > detection_threshold)
        {
            ping_start_timestamp = measurements[i].timestamp;
            ping_start_detected = true;
        }

        /*
         * Add the new measurement to the stream.
         */
        data.push_back(measurements[i]);

        /*
         * If there is no ping detected, pop data off the front to ensure the
         * data buffer doesn't get too large. The stream only buffers a small
         * subsection of the past.
         */
        if (ping_start_detected == false &&
                data.size() > measurements_to_buffer)
        {
            data.pop_front();
        }
    }
}

/**
 * Determines if the stream contains a full ping.
 *
 * @return True if the stream contains a full ping. False otherwise.
 */
bool DataStream::has_ping()
{
    return ping_start_detected &&
            data.size() > 0 &&
            data[data.size() - 1].timestamp >=
                    ping_start_timestamp + ping_length_s;
}

/**
 * Gets the start of ping timestamp.
 *
 * @return The start of ping timestamp;
 */
float DataStream::get_ping_start_timestamp()
{
    return ping_start_timestamp;
}
