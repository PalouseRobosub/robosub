#include "DataStreamServer.h"
#include "DataStream.h"
#include "AnalogPacket.h"
#include "MetadataPacket.h"
#include "robosub/HydrophonePingStamped.h"
#include "robosub/HydrophoneMetadata.h"

#include <ros/ros.h>

/**
 * Main entry point to application.
 *
 * @return Zero upon success.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "hydrophone_data_server");
    ros::NodeHandle np("~");
    ros::NodeHandle nh;

    int port_number;
    if (np.getParam("tcp_port", port_number) == false)
    {
        ROS_INFO("Failed to get ~tcp_port parameter. Defaulting to 8080.");
        port_number = 8080;
    }

    ROS_FATAL_COND(port_number > UINT16_MAX, "Port number too large.");
    ROS_FATAL_COND(port_number < 1024, "Port number too small.");

    DataStreamServer server;
    ROS_FATAL_COND(server.init(port_number) == fail,
            "Failed to initialize server on port %d.", port_number);

    /*
     * Set up the hydrophone data publishers.
     */
    ros::Publisher data_pub =
            nh.advertise<robosub::HydrophonePingStamped>("hydrophone/samples",
                                                         1);
    ros::Publisher metadata_pub =
            nh.advertise<robosub::HydrophoneMetadata>("hydrophone/metadata", 1);

    /*
     * All hydrophones are composed of a data stream channel. When a ping is
     * detected among the channels, a cross correlation is performed in order
     * to find the time delay of the signals.
     */
    DataStream channel[4];

    char buf[50000];

    while (ros::ok())
    {
        /*
         * Send a trigger to start sampling the hydrophones.
         */
        ros::Time sample_start = ros::Time::now();
        server.send_start_trigger();

        /*
         * The firmware samples continuously for approximately 1 second. Kill
         * the time before trying to read the data packets.
         */
        ros::Duration(0.5).sleep();

        /*
         * The first packet the raspberry pi sends is metadata about the current
         * processor state (aka clock rate). This tells us what each cycle
         * counter stamp on data points corresponds to in terms of real time.
         * It also tells us how long the raspberry pi took to sample the data.
         */
        if (server.get_packet(buf, MetadataPacket::packet_size))
        {
            ROS_FATAL("Failed to read metadata packet.");
            return -1;
        }

        MetadataPacket info_packet(buf, MetadataPacket::packet_size);
        float acquisition_start_time_s = info_packet.get_start_time();
        const float processor_frequency = info_packet.get_processor_speed();

        /*
         * Calculate the period of a processor cycle to inform the channels.
         * This will inform the packet parsing of how to convert processor
         * cycles to actual time.
         */
        const float cycle_counter_scale = 1.0 / processor_frequency;

        /*
         * Log metadata into a message we transmit later.
         */
        robosub::HydrophoneMetadata metadata_msg;
        metadata_msg.processor_frequency = processor_frequency;
        metadata_msg.processor_temperature = info_packet.get_temperature();
        metadata_msg.acquisition_time_system_cycles =
                info_packet.get_sample_time();
        metadata_msg.acquisition_time_clock_cycles =
                info_packet.get_sampling_ticks() * cycle_counter_scale;
        metadata_msg.start_time = acquisition_start_time_s;

        /*
         * Collect all of the data packets.
         */
        const uint32_t packet_count = info_packet.get_data_packet_count();
        for (uint32_t i = 0; i < packet_count; ++i)
        {
            if (server.get_packet(buf, AnalogPacket::packet_size))
            {
                ROS_FATAL("Read invalid analog packet on %d", i);
                return -1;
            }

            AnalogPacket packet(buf, AnalogPacket::packet_size, cycle_counter_scale);

            auto ch1 = packet.get_data_channel(0);
            auto ch2 = packet.get_data_channel(1);
            auto ch3 = packet.get_data_channel(2);
            auto ch4 = packet.get_data_channel(3);
            channel[0].insert(ch1);
            channel[1].insert(ch1);
            channel[2].insert(ch2);
            channel[3].insert(ch3);
        }

        ROS_INFO("Got all packets.");

        /*
         * Parse data for the start of the ping.
         */
        float ping_start_time;
        bool ping_detected = false;
        for (size_t i = 0; i < 4; ++i)
        {
            if (channel[i].has_ping() &&
                    (channel[i].get_ping_start_timestamp() < ping_start_time ||
                     ping_detected == false))
            {
                ping_start_time = channel[i].get_ping_start_timestamp();
                ping_detected = true;
            }
        }

        /*
         * If the start is found in the data, set the next future trigger time.
         * If no start was found, just set the trigger for 500ms in the future.
         */
        float ping_delta_s;
        if (ping_detected)
        {
            ping_delta_s = ping_start_time - acquisition_start_time_s;

            /*
             * If all of the channels indicate they have a full ping, perform a
             * cross correlation to find the time delay in the signal channels.
             */
            if (channel[0].has_ping() && channel[1].has_ping() &&
                channel[2].has_ping() && channel[3].has_ping())
            {
                /*
                 * Perform the cross correlation against the reference channel.
                 * This process is only responsible for gathering channels and
                 * creating the data for the cross-correlator. Prepare the data
                 * from the streamed information, kick off the asynchronous
                 * cross correlation, and continue with maintaining the data
                 * streams.
                 */
                robosub::HydrophonePingStamped sample_msg;

                sample_msg.header.stamp = ros::Time::now();

                channel[0].get_measurements(sample_msg.channel[0].data,
                        sample_msg.channel[0].time);
                channel[1].get_measurements(sample_msg.channel[1].data,
                        sample_msg.channel[1].time);
                channel[2].get_measurements(sample_msg.channel[2].data,
                        sample_msg.channel[2].time);
                channel[3].get_measurements(sample_msg.channel[3].data,
                        sample_msg.channel[3].time);

                data_pub.publish(sample_msg);
            }
            else
            {
                /*
                 * If not all channels detected a ping, there may be something
                 * wrong in ping detection. Log the event as a warning. We
                 * should be locked on to the ping after the first few sample
                 * cycles.
                 */
                ROS_WARN("Not all channels detected a ping.");
            }
        }
        else
        {
            ping_delta_s = 0.5;
        }

        metadata_pub.publish(metadata_msg);

        /*
         * Pings come every 2 seconds. Schedule slightly before then to
         * guarantee we collect the ping.
         */
        ros::Time next_ping_time = sample_start + ros::Duration(ping_delta_s) +
                ros::Duration(1.85);

        if (next_ping_time < ros::Time::now())
        {
            ROS_WARN("Next ping is already going!");
        }
        else
        {
            //ros::Duration(next_ping_time - ros::Time::now()).sleep();
        }

        ROS_INFO("Waiting for next cycle.");
        ros::Duration(5).sleep();
        ROS_INFO("Moving to next cycle.");
    }
}
