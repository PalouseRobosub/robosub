#include <ros/ros.h>
#include "utility/serial.hpp"
#include "robosub/thruster.h"
#include "movement/maestro_thruster_driver.h"
#include <string>
#include <map>
#include <vector>

typedef struct thruster_info
{
    std::string name;
    uint8_t channel;
}Thruster_info;

using namespace rs;
Serial serial;
MaestroThrusterDriver thrusterController;
std::vector<Thruster_info> mThruster_info;

void Callback (const robosub::thruster::ConstPtr& msg)
{
    int result = 0;
    for (unsigned int i = 0; i < msg->data.size(); i++)
    {
        result = thrusterController.set(msg->data[i],
                                        mThruster_info[i].channel);
        if(result != 0)
        {
            ROS_ERROR_STREAM("Setting speed of thrusters failed.");
        }
    }
}

void zeroThrusterSpeeds()
{
    int result = 0;
    for (unsigned int i = 0; i < mThruster_info.size(); i++)
    {
        result = thrusterController.set(0, mThruster_info[i].channel);
        if(result != 0)
        {
            ROS_ERROR_STREAM("Setting speed of thrusters failed.");
        }
    }
}

int main(int argc, char **argv)
{
    std::string thruster_port;
    std::map<uint8_t, double> max_speeds;
    ros::init(argc, argv, "thruster");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("thruster", 1, Callback);

    if(!n.getParam("ports/thruster", thruster_port))
    {
        ROS_FATAL("no serial port specified, exiting!");
        exit(1);
    }

    serial.Open(thruster_port.c_str(), B9600);

    //Get the ports and names of the maestro thrusters (from coblat.yaml)
    XmlRpc::XmlRpcValue thruster_settings;
    ros::param::get("thrusters/mapping", thruster_settings);

    for(int i = 0; i < thruster_settings.size(); ++i)
    {
        ROS_DEBUG_STREAM("thrusters["<< i << "][name]:    " <<
                         thruster_settings[i]["name"]);
        ROS_DEBUG_STREAM("thrusters["<< i << "][channel]: " <<
                         thruster_settings[i]["channel"]);
        Thruster_info one_thruster;
        one_thruster.name =
                        static_cast<std::string>(thruster_settings[i]["name"]);
        one_thruster.channel =
                             static_cast<int>(thruster_settings[i]["channel"]);
        mThruster_info.push_back(one_thruster);
    }

    thrusterController.init(&serial);

    ros::spin();

    //set speed of thrusters to zero when done
    zeroThrusterSpeeds();

    return 0;
}
