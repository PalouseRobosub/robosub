/**
 * @author Ryan Summers
 * @date 07-02-2017
 *
 * @brief Parsing class for the analog measurement packets transmitted over UDP
 *        by the raspberry Pi.
 */

#ifndef ANALOG_PACKET_H
#define ANALOG_PACKET_H

#include "AnalogMeasurement.h"

#include <unistd.h>
#include <cstdint>
#include <vector>

using std::vector;

/**
 * This class is designed for decoding the different elements present in the
 * analog packet transmitted by the Raspberry pi. These packets have a very
 * specific format (parenthesis indicate bit count):
 *
 * [ N * SamplePacket ]
 *
 *    Note: N is defined as AnalogPacket::samples_per_packet [firmware defined]
 */
class AnalogPacket
{
    /**
     * Describes a single analog sample (of all channels).
     *
     * Each sample packet consists of a 32-bit clock cycle counter and all four
     * analog samples (16 bits). The clock cycle counter is reset at the beginning
     * of sampling, so this value represents the number of clock cycles since
     * sampling began.
     */
    class SamplePacket
    {
        /**
         * Describes the number of data channels in a single sample packet.
         */
        static constexpr size_t samples_per_packet = 4;

    public:
        /**
         * Describes the size of each sample packet in bytes. Each
         * sample packet is composed of a 64 bit cycle counter and the
         * 4 16-bit analog measurements.
         */
        static constexpr size_t packet_size = (2 * samples_per_packet) + 8;

        SamplePacket(const char * buf, const uint32_t length);

        uint64_t get_cycle_counter();

        uint16_t get_sample(uint8_t channel_number);

    private:
        const char * const raw_data;

        const uint32_t raw_length;
    };

public:
    /**
     * Specifies the number of individual samples of a single channel that a
     * packet contains.
     */
    static constexpr uint16_t samples_per_packet = 500;

    /**
     * Specifies the number of bytes within an AnalogPacket.
     */
    static constexpr size_t packet_size = samples_per_packet * SamplePacket::packet_size;

    /**
     * Do not allow default construction.
     */
    AnalogPacket() = delete;

    AnalogPacket(const char *buf, const uint32_t length, const double cc_scale_factor);

    vector<AnalogMeasurement> get_data_channel(const uint8_t channel_number);

private:
    /**
     * Stores the raw data of the packet.
     */
    const char * const raw_data;

    /**
     * Stores the length of the raw data.
     */
    const uint32_t raw_length;

    /**
     * Stores the scale factor of cycle counts to seconds.
     */
    const double cycle_to_sec_scale;
};
#endif
