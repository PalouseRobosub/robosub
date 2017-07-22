#include "robosub/HydrophonePingStamped.h"

#include <fstream>
#include <string>
#include <ros/ros.h>
#include <unistd.h>

using namespace std;

bool run = false;
string csv_filename;
fstream csv_file;

void sample_callback(const robosub::HydrophonePingStamped::ConstPtr &msg)
{
    run = true;
    ROS_INFO("Got callback.");
    for (size_t i = 0; i < msg->channel[0].time.size(); ++i)
    {
        csv_file << msg->channel[0].time[i] << ", ";
        csv_file << msg->channel[0].data[i] << ", ";
        csv_file << msg->channel[0].data[i] << ", ";
        csv_file << msg->channel[0].data[i] << ", ";
        csv_file << msg->channel[0].data[i] << endl;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "hydrophone_stream_logger");

    ros::NodeHandle nh;

    if (ros::param::get("~file", csv_filename) == false)
    {
        ROS_WARN("~file Output parameter not set. Default: output.csv");
        csv_filename = "output.csv";
    }

    char cwd[500];
    getcwd(cwd, 500);
    ROS_INFO("CWD: %s", cwd);
    csv_file.open(csv_filename.c_str(), fstream::out);
    csv_file.precision(15);
    csv_file << fixed;
    csv_file << "Time, Channel One, Channel Two, Channel Three, Channel Four" << endl;

    ros::Subscriber hydrophone_sub = nh.subscribe("hydrophone/samples", 1,
            sample_callback);

    while (!run && ros::ok())
    {
        ros::spinOnce();
    }

    csv_file.close();

    return 0;
}
