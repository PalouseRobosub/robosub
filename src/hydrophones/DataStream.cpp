#include "DataStream.h"

#include "AnalogPacket.h"
#include "robosub/HydrophoneSamples.h"

#include <cstdint>

/**
 * Constructor.
 *
 * @param channel The channel number this datastream represents.
 */
DataStream::DataStream(const uint8_t channel) :
    channel_number(channel),
    data(),
    ping_start_detected(false),
    ping_done_timestamp(0)
{
}

/**
 * Extract the data from internal buffering to a list of timestamp/value pairs
 * for the channel.
 *
 * @param packet The ROS message to extract data into.
 *
 * @return Success or fail.
 */
result_t DataStream::get_measurements(robosub::HydrophoneSamples &packet)
{
    /*
     * Convert all data from the deque into individual measurements and copy
     * them into the provided buffer.
     */
    packet.channel[channel_number].time.clear();
    packet.channel[channel_number].data.clear();

    for (uint32_t packet_num = 0; packet_num < data.size(); ++packet_num)
    {
        ChannelPacket &data_packet = data[packet_num];
        for (uint32_t i = 0; i < AnalogPacket::samples_per_packet; ++i)
        {
            packet.channel[channel_number].data.push_back(
                    data_packet.samples[i].sample);
            packet.channel[channel_number].time.push_back(
                    data_packet.samples[i].timestamp);
        }
    }

    /*
     * Remove the data from internal buffering until the deque is at most 5
     * packets long.
     */
    while (data.size() >= 5)
    {
        data.pop_front();
    }

    return success;
}

/**
 * Append packetized data into the stream.
 *
 * @param packet The packetized data to add to the stream.
 *
 * @return Success or fail.
 */
result_t DataStream::insert(AnalogPacket &packet)
{
    ChannelPacket channel_packet;
    channel_packet.sequence = packet.get_sequence_number();

    if (packet.get_channel(channel_number,
                           channel_packet.samples,
                           AnalogPacket::samples_per_packet) == fail)
    {
        return fail;
    }

    data.push_back(channel_packet);

    /*
     * Check the new data for the start of a ping.
     */
    if (ping_start_detected == false)
    {
        for (int32_t i = 0; i < AnalogPacket::samples_per_packet; ++i)
        {
            if (channel_packet.samples[i].sample > detection_threshold)
            {
                ping_start_detected = true;
                ping_done_timestamp = channel_packet.samples[i].timestamp +
                        ping_length_us;
                break;
            }
        }
    }

    /*
     * If the packet did not contain our ping and the deque is
     * larger than the allotment for when a ping has not been
     * detected, pop the oldest data off.
     */
    if (ping_start_detected == false && data.size() > 5)
    {
        data.pop_front();
    }

    return success;
}

/**
 * Determines if the stream contains a full ping.
 *
 * @return True if the stream contains a full ping. False otherwise.
 */
bool DataStream::has_ping()
{
    /*
     * If the most recently received data point is past the end
     * of the ping, a full ping has been captured in the deque.
     */
    if (ping_start_detected)
    {
        if (data[data.size() - 1].samples[
                AnalogPacket::samples_per_packet - 1].timestamp >=
            ping_done_timestamp)
        {
            return true;
        }
    }

    return false;
}
