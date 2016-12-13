#include "ros/ros.h"
#include "ros/console.h"
#include "utility/test_tools.hpp"
#include <string>

namespace rs
{
    SerialTB::SerialTB()
    {
        started = false;
    }

    SerialTB::~SerialTB()
    {
    }

    int SerialTB::Start(std::string &test_port, std::string &uut_port)
    {
        if(started == true)
        {
            ROS_ERROR("cannot start serial testbench, already started");
            return -1;
        }
        pid_t pid = getpid();
        test_port = "test_" + std::to_string(pid);
        uut_port = "uut_" + std::to_string(pid);
        std::string args = "pty,raw,echo=0,link=" + test_port +
               " pty,raw,echo=0,link=" + uut_port;
        socat_proc.Spawn("/usr/bin/socat", args.c_str());

        started = true;

        return 0;
    }

};
