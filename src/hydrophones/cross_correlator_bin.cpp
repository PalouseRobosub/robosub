#include <cfloat>
#include <vector>
#include <cstdint>
#include "robosub/HydrophonePingStamped.h"
#include "robosub/HydrophoneDeltas.h"

#include <ros/ros.h>
#include <vector>
#include <fstream>

using namespace std;

ros::Publisher delta_pub;

/**
 * Describes the geometric constraint on ping arrival time delay between any
 * two hydrophones in seconds.
 */
static constexpr float geometric_constraint_s = 0.001;

void csvwrite(vector<float> &convolution, string filename)
{
    fstream out_file;
    out_file.open(filename.c_str(), fstream::out);
    out_file.precision(15);
    out_file << fixed;
    out_file << "Convolution" << endl;
    for (size_t i = 0; i < convolution.size(); ++i)
    {
        out_file << convolution[i] << endl;
    }
}

/**
 * Calculates the average delta between vector arguments.
 *
 * @param vector The vector to calculate deltas of.
 *
 * @return The average delta.
 */
template <typename T>
double get_average_delta(vector<T> &data)
{
    double accumulator = 0;
    int count = 0;

    for (size_t i = 1; i < data.size(); ++i)
    {
        accumulator += data[i] - data[i - 1];
        count++;
    }

    return accumulator / count;
}

/**
 * Calculates the average value of a vector.
 *
 * @return The average value of a vector.
 */
template <typename T>
double get_average(vector<T> &data)
{
    double accumulator = 0;
    for (size_t i = 0; i < data.size(); ++i)
    {
        accumulator += data[i];
    }

    return accumulator / data.size();
}

/**
 * Computes the (constrained) cross-correlation of two signals.
 *
 * result = f(t) * g(t);
 *
 * @param[in] f The lhs input signal. Constant sampling frequency is assumed.
 * @param[in] g The rhs input signal. Constant sampling frequency is assumed.
 * @param[out] result The result of the convolution of f and g.
 * @param max_lag The maximum number of sample delayss after the beginning to
 *        compute the cross-correlation for (essentially, this allows the
 *        implementation to restrict the convolution result to physically
 *        possible values.)
 *
 * @return None.
 */
void xcorr(vector<float> &f, vector<float> &g, vector<float> &result, int max_lag)
{
    /*
     * Reverse g then convolve.
     */
    std::reverse(g.begin(), g.end());

    size_t result_size = f.size() + g.size() - 1;

    result.clear();
    result.resize(result_size);

    for (size_t i = 0; i < result_size; ++i)
    {
        /*
         * In order to reduce the computation of the convolution, restrict
         * the bounds of it to be in line with geometric constraints.
         */
        if (abs(static_cast<int>(f.size()) - static_cast<int>(i)) < max_lag)
        {
            const size_t start_index = (i > g.size() - 1)? i - (g.size() - 1) : 0;
            const size_t end_index = (i < f.size() - 1)? i : f.size() - 1;
            for (size_t j = start_index; j <= end_index; ++j)
            {
                result[i] += f[j] * g[i - j];
            }
        }
        else
        {
            result[i] = 0;
        }
    }
}

/**
 * Handles a hydrophone sample message.
 *
 * @param msg A pointer to the ROS message.
 *
 * @return None.
 */
void handle_samples(const robosub::HydrophonePingStamped::ConstPtr &msg)
{
    ROS_INFO("Got message");

    /*
     * Normalize all readings by subtracting the average.
     */
    vector<float> channel[4];
    for (size_t i = 0; i < 4; ++i)
    {
        vector<uint16_t> data = msg->channel[i].data;
        const float average = get_average(data);

        for (size_t j = 0; j < msg->channel[i].data.size(); ++j)
        {
            const float val = msg->channel[i].data[j] - average;
            channel[i].push_back(val);
        }
    }

    /*
     * Calculate the average sampling period.
     */
    vector<float> times = msg->channel[0].time;
    const double sample_period = get_average_delta(times);

    /*
     * Convolve the data channels against the reference channel. The max lag
     * corresponds with the geometric restriction on time delay.
     */
    vector<float> result[3];
    for (int i = 0; i < 3; ++i)
    {
        xcorr(channel[0], channel[i + 1], result[i], geometric_constraint_s / sample_period);
    }

    //csvwrite(result[0], "convolution1.csv");
    ROS_INFO("Finished convolution.");

    /*
     * Find the maximum location in the result vectors. Calculate the time of
     * flight difference. Note that an index of len(samples)-1 corresponds to
     * a zero time-of-flight difference.
     */
    double flight_difference[3] = {0};
    for (size_t i = 0; i < 3; ++i)
    {
        size_t max_index = 0;
        for (size_t j = 0; j < result[i].size(); ++j)
        {
            if (result[i][j] > result[i][max_index])
            {
                max_index = j;
            }
        }

        /*
         * Convert index to time of flight difference.
         */
        int sample_count_delta = static_cast<int32_t>(max_index) -
                (channel[i].size() - 1);

        flight_difference[i] = sample_count_delta * sample_period * -1;
    }

    /*
     * Publish the time of flight delays.
     */
    robosub::HydrophoneDeltas delta_msg;

    delta_msg.xDelta = ros::Duration(flight_difference[0]);
    delta_msg.yDelta = ros::Duration(flight_difference[1]);
    delta_msg.zDelta = ros::Duration(flight_difference[2]);

    delta_pub.publish(delta_msg);
}

/**
 * Main entry point to the program.
 *
 * @return Zero upon success.
 */
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "hydrophone_correlator");

    ros::NodeHandle n;

    delta_pub = n.advertise<robosub::HydrophoneDeltas>("hydrophone/deltas", 1);

    ros::Subscriber data_sub = n.subscribe("hydrophone/samples", 1, &handle_samples);

    ros::spin();

    return 0;
}
