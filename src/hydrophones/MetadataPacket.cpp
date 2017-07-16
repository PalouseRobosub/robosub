#include "MetadataPacket.h"

#include <cstdint>
#include <cstring>
#include <arpa/inet.h>

MetadataPacket::MetadataPacket(const char * buf, const uint32_t len) :
    raw_data(buf),
    raw_length(len)
{
}

/**
 * Gets the processor speed.
 *
 * @return Firmware processor speed in Hz.
 */
uint32_t MetadataPacket::get_processor_speed()
{
    uint32_t processor_hz;
    memcpy(&processor_hz, raw_data, 4);

    return ntohl(processor_hz);
}

/**
 * Gets the processor temperature.
 *
 * @return The firmware processor temperature in degrees C.
 */
float MetadataPacket::get_temperature()
{
    uint32_t temperature_milliC;

    memcpy(&temperature_milliC, &raw_data[4], 4);

    return static_cast<float>(ntohl(temperature_milliC)) / 1000;
}

/**
 * Gets the system tick start of sampling.
 *
 * @return The firmware system tick start time in seconds (relative).
 */
float MetadataPacket::get_start_time()
{
    uint32_t start_time_us;
    memcpy(&start_time_us, &raw_data[8], 4);

    return static_cast<float>(ntohl(start_time_us)) / 1000000.0;
}

/**
 * Gets the system tick duration of sampling.
 *
 * @return The firmware system tick sampling duration in seconds.
 */
float MetadataPacket::get_sample_time()
{
    uint32_t sampling_time_us;
    memcpy(&sampling_time_us, &raw_data[12], 4);

    return static_cast<float>(ntohl(sampling_time_us)) / 1000000.0;
}

/**
 * Gets the number of processor cycles used for sampling.
 *
 * @return The number of firmware processor clock cycles during sampling.
 */
uint32_t MetadataPacket::get_sampling_ticks()
{
    uint32_t sampling_ticks;
    memcpy(&sampling_ticks, &raw_data[16], 4);

    return ntohl(sampling_ticks);
}

/**
 * Gets the number of data packets following the metadata packet.
 *
 * @return The number of packets to be transmitted.
 */
uint32_t MetadataPacket::get_data_packet_count()
{
    uint32_t packet_count;
    memcpy(&packet_count, &raw_data[20], 4);

    return static_cast<float>(ntohl(packet_count));
}
