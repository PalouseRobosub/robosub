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
    data()
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
}

/**
 * Clears all data from internal buffering.
 *
 * @return None.
 */
void DataStream::clear()
{
    data.clear();
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
         * Add the new measurement to the stream.
         */
        data.push_back(measurements[i]);
    }
}

/**
 * Truncates the stream to a specific window of time.
 *
 * @param start_time The start time to truncate data before.
 * @param end_time The time to truncate data after.
 *
 * @return None.
 */
void DataStream::window(double start_time, double end_time)
{
    size_t first_index = 0;
    size_t last_index = data.size() - 1;

    for (size_t i = 0; i < data.size(); ++i)
    {
        if (data[i].timestamp < start_time)
        {
            first_index = i;
        }

        if (data[i].timestamp > end_time)
        {
            last_index = i;
            break;
        }
    }

    /*
     * Remove the trailing data and then the front of the deque to perserve
     * index numbers.
     */
    if (last_index != data.size() - 1)
    {
        data.erase(data.begin() + last_index, data.end());
    }

    if (first_index != 0)
    {
        data.erase(data.begin(), data.begin() + first_index);
    }
}

/**
 * Gets the start of ping timestamp.
 *
 * @param[out] start_time The detected start time if a ping was detected.
 *
 * @return True if the ping start was detected.
 */
bool DataStream::get_ping_start_time(double &start_time)
{
    if (data.size() == 0)
    {
        return false;
    }

    float accumulator = data[0].sample;
    bool tracking_ping = false;
    bool start_detected = false;

    for (size_t i = 1; i < data.size(); ++i)
    {
        /*
         * Perform peak detection on the data. If the data is higher than
         * the accumulator, update accumulator.
         */
        if (data[i].sample > accumulator)
        {
            accumulator = data[i].sample;
        }
        else
        {
            /*
             * If data was not above the accumulator, decay the accumulator
             * value by subtracting a portion of the accumulator and sample
             * difference.
             */
            float cycles = (data[i].timestamp - data[i - 1].timestamp) /
                    period_s;

            accumulator -= (1 - cycles * rectification_decay_per_cycle) *
                    (accumulator - data[i].sample);
        }

        /*
         * If the accumulator hits the threshold, a start condition was found.
         */
        if (accumulator > detection_threshold)
        {
            start_detected = true;

            /*
             * If we aren't already tracking a ping, log the start timestamp
             * and mark that we are now tracking the ping.
             */
            if (!tracking_ping)
            {
                start_time = data[i].timestamp;
                tracking_ping = true;
            }
        }
        else
        {
            tracking_ping = false;
        }
    }

    return start_detected;
}
