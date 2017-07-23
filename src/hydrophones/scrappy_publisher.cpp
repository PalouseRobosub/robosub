#include "robosub/HydrophonePingStamped.h"
#include <string>
#include <fstream>

#include <ros/ros.h>

using namespace std;

robosub::HydrophonePingStamped data_msg;

void csvload(string filename)
{
    ifstream input_file(filename);
    {
        string line;
        getline(input_file, line);
        stringstream ss(line);
        while (ss.rdbuf()->in_avail())
        {
            float data;
            ss >> data;
            data_msg.channel[0].time.push_back(data);
        }
    }

    for (size_t i = 0; i < 4; ++i)
    {
        string line;
        getline(input_file, line);
        stringstream ss(line);
        while (ss.rdbuf()->in_avail())
        {
            float data;
            ss >> data;
            data_msg.channel[i].data.push_back(data);
        }
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "stupid_pubber");

    string filename = "data.csv";

    ros::NodeHandle n;
    ros::Publisher data_pub = n.advertise<robosub::HydrophonePingStamped>("hydrophone/samples", 1);

    /*
     * Load data files.
     */
    csvload(filename);

    while (ros::ok())
    {
        data_pub.publish(data_msg);
        ros::Duration(5).sleep();
    }

    return 0;
}
