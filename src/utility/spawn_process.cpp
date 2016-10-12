#include <unistd.h>
#include <errno.h>
#include <string>
#include <signal.h>
#include "ros/ros.h"
#include "ros/console.h"
#include "utility/test_tools.hpp"

namespace rs
{
    SpawnProcess::SpawnProcess()
    {
        m_pid = -1;
        spawned = false;
    }
    void SpawnProcess::Spawn(std::string cmd, std::string args)
    {
        if(spawned == true) //if we've already forked our process
        {
            ROS_ERROR("cannot spawn process, already spawned");
            return;
        }

        m_pid = fork();
        if(m_pid < 0) //fork failed
        {
            ROS_FATAL_STREAM("failed to fork, error: " << strerror(errno));
            exit(1);
        }
        else if (m_pid  == 0) //child
        {
            ROS_DEBUG("spawning \"%s\" with args \"%s\"", cmd.c_str(), args.c_str());
            char *arg_ptrs[64];

            arg_ptrs[0] = const_cast<char*>(cmd.c_str());
            int i=1;
            if(args != "")
            {
                arg_ptrs[i] = strtok(const_cast<char*>(args.c_str()), " ");
                while(arg_ptrs[++i] = strtok(NULL, " "));
            }
            arg_ptrs[i] = 0;

            execvp(cmd.c_str(), arg_ptrs);
            ROS_FATAL_STREAM("failed to exec \"" << cmd << "\", error: " << strerror(errno));
            exit(1);
        }
        else //parent
        {
            ROS_DEBUG("Forked child, pid: %d", m_pid);
            //nothing to do
        }
        spawned = true;
    }

    SpawnProcess::~SpawnProcess()
    {
        //kill the child process when we go out of scope
        kill(m_pid, SIGTERM);
    }

};
