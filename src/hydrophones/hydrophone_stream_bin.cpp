#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string>
#include <time.h>

#include "DataStreamServer.h"
#include "AnalogPacket.h"

/*
 * Specifies the port number to listen to.
 */
static constexpr uint16_t port_number = 8080;

/*
 * fatal_error - wrapper for perror
 */
void fatal_error(string msg)
{
    perror(msg.c_str());
    exit(1);
}

int main(int argc, char **argv)
{
    DataStreamServer server;

    if (server.init(port_number) == fail)
    {
        fatal_error("Failed to initialize server.");
    }

    uint64_t received_bytes = 0;
    clock_t start = clock();
    struct timespec clock_start;
    clock_gettime(CLOCK_MONOTONIC, &clock_start);
    int i = 0;
    char buf[50000];

    /*
     * All hydrophones are composed of a data stream channel. When a ping is
     * detected among the channels, a cross correlation is performed against
     * the reference channel in order to find the time delay of the signals.
     */
    DataStream reference(0);
    DataStream channel[3]{{1}, {2}, {3}};

    /*
     * Stores the packet sequence number to detect out-of-sequence packets.
     */
    uint32_t sequence_number = 0;
    bool sequence_started = false;

    while (1)
    {
        int packet_len = server.get_packet(buf, 50000);
        if (packet_len < 0)
        {
            fatal_error("Failed to get packet.");
        }

        AnalogPacket packet(buf, packet_len);
        if (packet.parse() == fail)
        {
            fatal_error("Failed to parse packet.");
        }

        /*
         * Check the packet sequence number for a potential packet drop. If a
         * drop is detected, log it. Additionally, log any out of order packet
         * receipts. If this is a prevalent issue, we will need to modify the
         * data streams to accomodate for them.
         */
        if (sequence_started)
        {
            if (packet.sequence_number() != sequence_number + 1)
            {
                printf("Got out of sequence packet: # %d Timestamp: %llu\n",
                       packet.sequence_number(),
                       packet.timestamp());
            }
            sequence_number = packet.sequence_number();
        }
        else
        {
            sequence_number = packet.sequence_number();
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
            start_xcorr(reference, channel[0], channel[1], channel[2]);
        }

        //Debug information
        //TODO: Remove before commit.
        received_bytes += n;
        if (i++ % 10 == 0)
        {
            struct timespec clock_now;
            clock_gettime(CLOCK_MONOTONIC, &clock_now);
            float now = (clock_now.tv_sec - clock_start.tv_sec) + (clock_now.tv_nsec - clock_start.tv_nsec) / 1000000000.0;
            printf("%lf: %lf Mbps\r", now, received_bytes * 8 / 1000000 / now);
        }
    }
}
