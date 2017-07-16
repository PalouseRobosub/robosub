#ifndef METADATA_PACKET_H
#define METADATA_PACKET_H

#include <cstdint>
#include <unistd.h>

class MetadataPacket
{
public:
    /**
     * Describes the length of a metadata packet in bytes.
     */
    static constexpr size_t packet_size = 24;

    MetadataPacket(const char * buf, const uint32_t len);

    uint32_t get_processor_speed();

    float get_temperature();

    float get_start_time();

    float get_sample_time();

    uint32_t get_sampling_ticks();

    uint32_t get_data_packet_count();

private:
    const char * const raw_data;
    const uint32_t raw_length;
};

#endif
