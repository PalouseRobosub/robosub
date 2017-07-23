/**
 * @author Ryan Summers
 * @date 07/22/2017
 *
 * @brief Collects hydrophone data from a raspberry pi data server and publishes
 *        sample/time information.
 */

#include "AnalogPacket.h"
#include "DataStream.h"
#include "DataStreamServer.h"
#include "MetadataPacket.h"
#include "robosub/HydrophoneMetadata.h"
#include "robosub/HydrophonePingStamped.h"

#include <fstream>
#include <ros/ros.h>

using namespace std;

/**
 * Specifies the approximate sampling frequency of the raspberry pi.
 */
static constexpr uint32_t sampling_frequency = 600000;

/**
 * Specifies the amount of time that we need to wake up by before a ping starts.
 */
static constexpr float preping_wakeup_s = 0.025;

/**
 * Specifies the number of seconds before the ping start detection time that
 * preceeding data samples should be truncated to.
 */
static constexpr float preping_truncate_s = 0.001;

/**
 * Specifies the number of seconds involved in an actual ping.
 */
static constexpr float ping_duration_s = 0.004;

/**
 * Defines the maximum time delay between any two hydrophones receiving the
 * same ping and is defined by the mechanical mounting of the hydrophones.
 */
static constexpr float geometric_constraint_s = 0.001;

/**
 * Specifies the number of seconds after the ping start detection time that
 * proceeding data samples should be truncated to.
 */
static constexpr float postping_truncate_s = ping_duration_s +
        preping_truncate_s + geometric_constraint_s;

/**
 * Specifies the period between the start of two pings in seconds.
 */
static constexpr float ping_period_s = 2.0;

/**
 * Specifies the amount of time to sample for time-sync operations (in secs).
 */
static constexpr float time_sync_sample_duration_s = ping_period_s * 1.2;

/**
 * Specifies the amount of time to sample for correlation operations (in secs).
 */
static constexpr float correlation_sample_duration_s = 0.150;

/**
 * Specifies the number of samples during time-sync operations.
 */
static constexpr uint32_t time_sync_samples = time_sync_sample_duration_s /
        (1.0 / sampling_frequency);

/**
 * Specifies the number of samples during hydrophone-sample operations.
 */
static constexpr uint32_t correlation_samples = correlation_sample_duration_s /
        (1.0 / sampling_frequency);

void load_simulation_data(string filename, vector<AnalogMeasurement::SampleType> &simulation_samples)
{
    simulation_samples.clear();

    ifstream input_file(filename);

    string line;
    while (input_file.good())
    {
        getline(input_file, line);
        stringstream ss(line);
        int data;
        ss >> data;
        simulation_samples.push_back(data);
    }

    fstream output_file;
    output_file.open("wtf.csv", fstream::out);
    for (size_t i = 0; i < simulation_samples.size(); ++i)
    {
        output_file << simulation_samples[i] << endl;
    }

    output_file.close();
}

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

    /*
     * Load the port number to use.
     */
    int port_number;
    if (np.getParam("tcp_port", port_number) == false)
    {
        ROS_INFO("Failed to get ~tcp_port parameter. Defaulting to 8080.");
        port_number = 8080;
    }

    bool filter = true;

    /*
     * Load the IIR filter coefficients.
     */
    vector<double> all_coefficients;
    vector<vector<double>> coefficients;
    if (np.getParam("iir_coefficients", all_coefficients) == false)
    {
        ROS_WARN("Did not load IIR coefficients! Filtering will not be done.");
        filter = false;
    }
    else
    {
        if (all_coefficients.size() % 6)
        {
            ROS_FATAL("Coefficients must be a multiple of 6!");
            return -1;
        }

        /*
         * Split apart the coefficients into each filter order coefficients.
         */
        for (size_t i = 0; i < all_coefficients.size() / 6; ++i)
        {
            vector<double> current_coefficients;
            for (size_t j = 0; j < 6; ++j)
            {
                current_coefficients.push_back(all_coefficients[i * 6 + j]);
            }

            coefficients.push_back(current_coefficients);
        }
    }

    /*
     * Check to see if we should write out the time-synchronization data to
     * a file and exit. This mode is used for debugging signals provided by
     * the sampling hardware.
     */
    bool log_long_data = false;
    if (np.getParam("log_data", log_long_data))
    {
        if (log_long_data)
        {
            ROS_INFO("Logging long data sample and exitting.");
        }
    }

    /*
     * Check to see if the IIR filter should be applied.
     */
    bool do_not_filter = false;
    if (np.getParam("no_filter", do_not_filter))
    {
        if (do_not_filter)
        {
            ROS_INFO("DoNotFilter flag found. Will not filter.");
            filter = false;
        }
    }

    string simulation_filename;
    bool simulating_data = false;
    vector<AnalogMeasurement::SampleType> simulation_data;
    if (np.getParam("simulation_file", simulation_filename))
    {
        ROS_INFO("Loading simulation file.");
        simulating_data = true;
        load_simulation_data(simulation_filename, simulation_data);
    }

    /*
     * Verify the port parameter.
     */
    ROS_FATAL_COND(port_number > UINT16_MAX, "Port number too large.");
    ROS_FATAL_COND(port_number < 1024, "Port number too small.");

    DataStreamServer server;
    ROS_FATAL_COND(server.init(port_number),
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

    bool synchronization_acquired = false;

    while (ros::ok())
    {
        /*
         * Send a trigger to start sampling the hydrophones.
         */
        ros::Time sample_start = ros::Time::now();

        /*
         * If time synchronization (locking with the ping time) hasn't been
         * acquired, we need to do an extended sampling duration to find the
         * start of the ping.
         */
        if (!synchronization_acquired)
        {
            server.send_start_trigger(time_sync_samples);
        }
        else
        {
            server.send_start_trigger(correlation_samples);
        }

        /*
         * The first packet the raspberry pi sends is metadata about the current
         * processor state (aka clock rate). This tells us what each cycle
         * counter stamp corresponds to in terms of real time. It also tells
         * us how long the raspberry pi took to sample the data.
         */
        if (server.get_packet(buf, MetadataPacket::packet_size))
        {
            ROS_FATAL("Failed to read metadata packet.");
            return -1;
        }

        MetadataPacket info_packet(buf, MetadataPacket::packet_size);
        float acquisition_start_time_s = info_packet.get_start_time();
        const double processor_frequency = info_packet.get_processor_speed();

        /*
         * Calculate the period of a processor cycle to inform the channels.
         * This will inform the packet parsing of how to convert processor
         * cycles to actual time.
         */
        const double cycle_counter_scale = 1.0 / processor_frequency;

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
         * Clear the data channels of any data from previous sampling cycles.
         */
        for (auto i = 0; i < 4; ++i)
        {
            channel[i].clear();
        }

        /*
         * Collect all of the data packets.
         */
        const uint32_t packet_count = info_packet.get_data_packet_count();
        for (uint32_t i = 0; i < packet_count; ++i)
        {
            if (server.get_packet(buf, AnalogPacket::packet_size))
            {
                ROS_FATAL("Read invalid analog packet on packet # %d.", i);
                return -1;
            }

            AnalogPacket packet(buf, AnalogPacket::packet_size, cycle_counter_scale);

            auto ch1 = packet.get_data_channel(0);
            auto ch2 = packet.get_data_channel(1);
            auto ch3 = packet.get_data_channel(2);
            auto ch4 = packet.get_data_channel(3);
            channel[0].insert(ch1);
            channel[1].insert(ch2);
            channel[2].insert(ch3);
            channel[3].insert(ch4);
        }

        ROS_INFO("Got all packets.");

        /*
         * If we're simulating data, we don't care what the raspberry pi sent.
         */
        if (simulating_data)
        {
            for (size_t i = 0; i < 4; i++)
            {
                channel[i].overwrite(simulation_data, 0, 4096);
            }
        }

        /*
         * If we haven't yet acquired sync, we need to filter out the 3
         * hydrophone frequencies from different quandrants. This is done by
         * applying a bandpass IIR filter centered arouned the target
         * frequency.
         *
         * TODO: Should we always filter the data anyways to ensure we didn't
         *       lock onto a bad signal?
         */
        if (!synchronization_acquired && filter)
        {
            for (size_t i = 0; i < 4; ++i)
            {
                for (size_t order = 0; order < coefficients.size(); ++order)
                {
                    if (channel[i].filter(coefficients[order]))
                    {
                        ROS_FATAL("Failed to filter.");
                        return -1;
                    }
                }
            }
        }

        /*
         * Parse data for the start of the ping.
         */
        double ping_start_time;
        bool ping_detected = false;
        for (size_t i = 0; i < 4; ++i)
        {
            /*
             * Get the latest ping start time from each channel.
             */
            double start_time = 0;
            if (channel[i].get_ping_start_time(start_time))
            {
                if (ping_detected == false ||
                        ping_start_time > start_time)
                {
                    ping_start_time = start_time;
                    ping_detected = true;
                }
            }
        }

        /*
         * Because of wierdness in the raspberry pi sampling, we can't do
         * cross-correlation on data during time-synchronization samples. The
         * sampling frequency gets a bit unstable and the data points are
         * not evenly spaced. Only publish messages during data captures of
         * synchronized data.
         */
        if ((ping_detected && synchronization_acquired) ||
                (log_long_data && !synchronization_acquired))
        {
            /*
             * If we're logging a synchronization point, just write it to
             * a file and exit.
             */
            if (log_long_data)
            {
                fstream output_file;
                output_file.open("long_output2.csv", fstream::out);

                vector<float> time;
                vector<int16_t> data[4];
                for (size_t i = 0; i < 4; ++i)
                {
                    channel[i].get_measurements(data[i], time);
                }

                output_file << fixed;
                output_file.precision(15);
                output_file << "time, ref, d1, d2, d3" << endl;
                for (size_t i = 0; i < time.size(); ++i)
                {
                    output_file << time[i];
                    for (size_t j = 0; j < 4; ++j)
                    {
                        output_file << ", " << data[j][i];
                    }
                    output_file << endl;
                }

                output_file.close();

                ROS_INFO("Logged long data to long_output.csv");
                return 0;
            }

            /*
             * Otherwise, we need to truncate the channels and transmit the
             * data.
             */
            for (auto i = 0; i < 4; ++i)
            {
                /*
                 * Truncate the samples to 10ms before ping start detection
                 * and allow samples until 60ms after the end of the ping.
                 */
                channel[i].window(ping_start_time - preping_truncate_s,
                                  ping_start_time + postping_truncate_s);
            }

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

        /*
         * Publish the metadata about the samples.
         */
        metadata_pub.publish(metadata_msg);

        if (ping_detected)
        {
            /*
             * If the ping start time was detected, calculate when the next
             * ping should be.
             */
            ros::Time ping_time = sample_start +
                                    ros::Duration(ping_start_time);

            /*
             * Figure out when the next ping should occur in the future.
             * Multiple pings could have come during our sampling cycle. We
             * want to wake up about 25ms before the ping.
             */
            while (ping_time < (ros::Time::now() +
                                ros::Duration(preping_wakeup_s)))
            {
                ping_time += ros::Duration(ping_period_s);
            }

            /*
             * Next, sleep until shortly before the next ping.
             */
            ros::Duration sleep_time = (ping_time -
                    ros::Duration(preping_wakeup_s)) - ros::Time::now();

            ROS_FATAL_COND(sleep_time.toSec() < 0, "Sleep duration negative!");

            sleep_time.sleep();
        }

        /*
         * If we just lost synchronization, log it.
         */
        ROS_WARN_COND(synchronization_acquired && !ping_detected,
                "Did not detect the ping after synchronized!");

        /*
         * Update our internal state to indicate that we found time sync.
         */
        synchronization_acquired = ping_detected;
    }
}
