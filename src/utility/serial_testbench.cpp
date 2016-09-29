#include "ros/ros.h"
#include "ros/console.h"
#include "utility/serial_testbench.hpp"

namespace rs
{
    SerialTB::SerialTB()
    {
        started = false;
    }

    SerialTB::~SerialTB()
    {
    }

    std::string SerialTB::Start(std::string UUT_port)
    {
        if(started == true)
        {
            ROS_ERROR("cannot start serial testbench, already started");
            return std::string();
        }
        pid_t pid = getpid();
        std::string testing_port = "test_" + std::to_string(pid);
        std::string args = "pty,raw,echo=0,link=" + testing_port +
               "pty,raw,echo=0,link=" + UUT_port;
        socat_proc.Spawn("/usr/bin/socat", args.c_str());

        started = true;

        return testing_port;
    }

};
