#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string>
#include <time.h>

#include "DataStreamServer.h"
#include "DataStream.h"
#include "AnalogPacket.h"
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
    if (np.getParam("udp_port", port_number) == false)
    {
        ROS_INFO("Failed to get ~udp_port parameter. Defaulting to 8080.");
        port_number = 8080;
    }

    ROS_FATAL_COND(port_number < UINT16_MAX, "Port number too large.");

    DataStreamServer server;
    ROS_FATAL_COND(server.init(port_number) == fail,
            "Failed to initialize server on port %d.", port_number);

    /*
     * Set up the hydrophone data publisher.
     */
    ros::Publisher data_pub =
            nh.advertise<robosub::HydrophoneStream>("hydrophone/samples", 1);

    /*
     * All hydrophones are composed of a data stream channel. When a ping is
     * detected among the channels, a cross correlation is performed against
     * the reference channel in order to find the time delay of the signals.
     */
    DataStream reference(0);
    DataStream channel[3]{{1}, {2}, {3}};

    uint32_t sequence_number = 0;
    bool sequence_started = false;
    char buf[50000];

    while (ros::ok())
    {
        int packet_len = server.get_packet(buf, 50000);
        ROS_FATAL_COND(packet_len < 0, "Failed to get packet.");

        AnalogPacket packet(buf, packet_len);
        ROS_FATAL_COND(packet.parse() == fail, "Failed to parse packet.");

        /*
         * Check the packet sequence number for a potential packet drop. If a
         * drop is detected, log it. Additionally, log any out of order packet
         * receipts. If this is a prevalent issue, we will need to modify the
         * data streams to accomodate for them.
         */
        if (sequence_started)
        {
            if (packet.get_sequence_number() != sequence_number + 1)
            {
                ROS_INFO("Got out of sequence packet: %d\n",
                        packet.get_sequence_number());
            }
            sequence_number = packet.get_sequence_number();
        }
        else
        {
            sequence_number = packet.get_sequence_number();
            sequence_started = true;
        }

        /*
         * Store the extracted data into the channel streams. The Data Stream
         * will store the blocks of data, and timestamps for figuring out the
         * time step for each data unit.
         */
        reference.insert(packet);
        channel[0].insert(packet);
        channel[1].insert(packet);
        channel[2].insert(packet);

        /*
         * If all of the channels indicate they have a full ping, perform a
         * cross correlation to find the time delay in the signal channels.
         */
        if (reference.has_ping() && channel[0].has_ping() &&
            channel[1].has_ping() && channel[2].has_ping())
        {
            /*
             * Perform the cross correlation against the reference channel.
             * This process is only responsible for gathering channels and
             * creating the data for the cross-correlator. Prepare the data
             * from the streamed information, kick off the asynchronous cross
             * correlation, and continue with maintaining the data streams.
             */
            robosub::HydrophoneSamples sample_msg;
            ROS_FATAL_COND(reference.get_measurements(sample_msg) == fail,
                    "Failed to extract samples from reference channel.");
            ROS_FATAL_COND(channel[0].get_measurements(sample_msg) == fail,
                    "Failed to extract samples from channel 0.");
            ROS_FATAL_COND(channel[1].get_measurements(sample_msg) == fail,
                    "Failed to extract samples from channel 1.");
            ROS_FATAL_COND(channel[2].get_measurements(sample_msg) == fail,
                    "Failed to extract samples from channel 2.");

            data_pub.publish(sample_msg);
        }
    }
}
