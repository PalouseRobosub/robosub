/**
 * @author Ryan Summers
 * @date 07-02-2017
 *
 * @brief Parsing class for the analog measurement packets transmitted over UDP
 *        by the raspberry Pi.
 */
#ifndef ANALOG_PACKET_H
#define ANALOG_PACKET_H

#include <cstdint>
#include "rs_types.h"
#include "AnalogMeasurement.h"

/**
 * This class is designed for decoding the different elements present in the
 * analog packet transmitted by the Raspberry pi. These packets have a very
 * specific format (parenthesis indicate bit count):
 * [ (32) Sequence Counter | (64) Timestamp | 182 x (64) AnalogMeasurementPacket ]
 *
 * The analog measurement packet is defined as:
 * [(16) measurement delay | (16) channel one | (16) channel two | (16) channel three | (16) channel_four]
 *     Note: The first analog measurement does not have a delay.
 */
class AnalogPacket
{
public:
    /**
     * Specifies the number of individual samples of a single channel that a
     * packet contains.
     */
    static constexpr uint16_t samples_per_packet = 182;

    AnalogPacket(const char *buf, const uint32_t length);

    result_t parse();

    result_t get_channel(const uint8_t channel_number,
                         AnalogMeasurement *result,
                         const uint32_t max_len);

    uint32_t get_sequence_number();

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
     * All packets have a sequence number (32-bit, rollover) to detect if
     * packets are dropped in transmission.
     */
    uint32_t sequence_number;
};
#endif
