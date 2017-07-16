#include "AnalogPacket.h"
#include "AnalogMeasurement.h"

#include <arpa/inet.h>
#include <cstring>
#include <cstdint>
#include <endian.h>

/**
 * Constructor.
 *
 * @param buf The pointer to the raw data.
 * @param len The length of raw data. Should be 1468.
 * @param cycle_scale The scale factor to convert processor cycles to seconds.
 */
AnalogPacket::AnalogPacket(const char *buf,
                           const uint32_t len,
                           const float cycle_scale) :
    raw_data(buf),
    raw_length(len),
    cycle_to_sec_scale(cycle_scale)
{
}

/**
 * Extract the measurements from a packet into an external buffer.
 *
 * @param channel_number The channel number to extract.
 *
 * @return Success or fail.
 */
vector<AnalogMeasurement> AnalogPacket::get_data_channel(
        const uint8_t channel_number)
{
    if (channel_number > 3)
    {
        return vector<AnalogMeasurement>();
    }

    vector<AnalogMeasurement> all_measurements;

    /*
     * Parse out analog measurements for the appropriate channel.
     */
    const char *raw_channel = raw_data;

    for (int i = 0; i < samples_per_packet;
                        ++i, raw_channel += SamplePacket::packet_size)
    {
        AnalogMeasurement measurement;

        SamplePacket sample_packet(raw_channel, SamplePacket::packet_size);

        /*
         * Extract the delta delays from each measurement to get the exact
         * timestamp for every sample.
         */
        measurement.timestamp = sample_packet.get_cycle_counter() *
                cycle_to_sec_scale;

        /*
         * Get the actual analog sample.
         */
        measurement.sample = sample_packet.get_sample(channel_number);

        all_measurements.push_back(measurement);
    }

    return all_measurements;
}

/**
 * Constructor.
 *
 * @param buf The raw data to utilize.
 * @param length The length of the raw data.
 */
AnalogPacket::SamplePacket::SamplePacket(const char * buf, const uint32_t length) :
    raw_data(buf),
    raw_length(length)
{
}

/**
 * Gets the processor cycle counter for the samples.
 *
 * @return The processor cycle counter.
 */
uint32_t AnalogPacket::SamplePacket::get_cycle_counter()
{
    uint32_t cycles;
    memcpy(&cycles, raw_data, 4);

    return ntohl(cycles);
}

/**
 * Gets an analog sample measurement.
 *
 * @param channel_number The measurement to extract.
 *
 * @return The analog sample for the specified channel number.
 */
uint16_t AnalogPacket::SamplePacket::get_sample(uint8_t channel_number)
{
    if (channel_number > AnalogPacket::SamplePacket::samples_per_packet)
    {
        return 0;
    }

    uint16_t sample;
    memcpy(&sample, &raw_data[4 + channel_number * 2], 2);

    return ntohs(sample);
}
