#include "AnalogPacket.h"
#include "AnalogMeasurement.h"

#include <arpa/inet.h>
#include <cstring>
#include <cstdint>

/**
 * Constructor.
 *
 * @param buf The pointer to the raw data.
 * @param len The length of raw data. Should be 1468.
 */
AnalogPacket::AnalogPacket(const char *buf, const uint32_t len) :
    raw_data(buf),
    raw_length(len)
{
}

/**
 * Parses analog packets into analog measurements per channel.
 *
 * @return Success or fail.
 */
result_t AnalogPacket::parse()
{
    if (raw_length != 12 + samples_per_packet * 10)
    {
        return fail;
    }

    /*
     * Extract the sequence number and time stamp.
     */
    uint32_t data;
    memcpy(&data, raw_data, 4);
    sequence_number = htonl(data);

    return success;
}

/**
 * Network order swap for 64 bit numbers.
 *
 * @note Taken from:
 *     https://stackoverflow.com/questions/3022552/is-there-any-standard-htonl-like-function-for-64-bits-integers-in-c
 *
 * @param value The value to convert.
 *
 * @return The proper system-specific value.
 */
uint64_t htonll(uint64_t value)
{
    int num = 42;
    if (*(char *)&num == 42)
    {
        uint32_t high_part = htonl((uint32_t)(value >> 32));
        uint32_t low_part = htonl((uint32_t)(value & 0xFFFFFFFFLL));
        return (((uint64_t)low_part) << 32) | high_part;
    }
    else
    {
        return value;
    }
}

/**
 * Extract the measurements from a packet into an external buffer.
 *
 * @param channel_number The channel number to extract.
 * @param result The location to store the extracted data.
 * @param max_len The maximum number of datapoints to extract into the buffer.
 *
 * @return Success or fail.
 */
result_t AnalogPacket::get_channel(const uint8_t channel_number,
                                   AnalogMeasurement *result,
                                   const uint32_t max_len)
{
    if (channel_number > 3)
    {
        return fail;
    }

    if (max_len < samples_per_packet)
    {
        return fail;
    }

    uint64_t long_data, timestamp;
    memcpy(&long_data, &raw_data[4], 8);
    timestamp = htonll(long_data);

    /*
     * Parse out analog measurements for the appropriate channel.
     */
    const char *raw_channel = &raw_data[12];
    for (int i = 0; i < samples_per_packet; ++i)
    {
        /*
         * Extract the delta delays from each measurement to get the exact
         * timestamp for every sample. The first does not have a delay.
         */
        if (i == 0)
        {
            result[i].timestamp = timestamp;
        }
        else
        {
            uint16_t delay = 0;
            memcpy(&delay, raw_channel, 2);
            result[i].timestamp = timestamp + ntohs(delay);
            raw_channel += 2;
        }

        /*
         * Copy the appropriate channel measurement
         */
        uint16_t measurement;
        memcpy(&measurement, &raw_channel[channel_number * 2], 2);
        raw_channel += 8;
    }

    return success;
}

uint32_t AnalogPacket::get_sequence_number()
{
    return sequence_number;
}
