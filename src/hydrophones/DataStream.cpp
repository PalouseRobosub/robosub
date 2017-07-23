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
void DataStream::get_measurements(vector<AnalogMeasurement::SampleType> &samples,
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
 * Filters the data contained in the stream through an IIR filter.
 *
 * @param coefficients The IIR filter coefficients. They should be in the order
 *        [b0, b1, b2, a0, a1, a2] such that aN is the
 *        denominator of the transfer function.
 *
 * @return Zero upon success.
 */
int DataStream::filter(vector<double> coefficients)
{
    /*
     * There should be 6 different coefficients.
     */
    if (coefficients.size() != 6)
    {
        return -1;
    }

    /*
     * First, normalize the coefficients by referencing coefficient a0.
     */
    const double reference_coefficient = coefficients[3];
    for (size_t i = 0; i < coefficients.size(); ++i)
    {
        coefficients[i] = coefficients[i] / reference_coefficient;
    }

    /*
     * Next, loop through all data points in the stream and apply the IIR
     * filter.
     */
    double z1 = 0, z2 = 0;
    for (size_t i = 0; i < data.size(); ++i)
    {
        const double old_value = data[i].sample;
        const double new_value = coefficients[0] * old_value + z1;

        /*
         * Update the internal IIR state variables.
         */
        z1 = coefficients[1] * old_value + z2 - coefficients[4] * new_value;
        z2 = coefficients[2] * old_value - coefficients[5] * new_value;

        /*
         * Update the data stream with the filtered data point.
         */
        data[i].sample = new_value;
    }

    return 0;
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

void DataStream::overwrite(vector<AnalogMeasurement::SampleType> &simulation_data, size_t right_shift, AnalogMeasurement::SampleType fill)
{
    for (size_t i = 0; i < right_shift && i < data.size(); ++i)
    {
        data[i].sample = fill;
    }

    for (size_t i = 0; i < simulation_data.size() && i + right_shift < data.size(); ++i)
    {
        data[i + right_shift].sample = simulation_data[i];
    }

    for (size_t i = simulation_data.size(); i < data.size(); ++i)
    {
        data[i].sample = fill;
    }
}
