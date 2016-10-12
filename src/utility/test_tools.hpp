#ifndef __TEST_TOOLS_HPP__
#define __TEST_TOOLS_HPP__

#include "ros/ros.h"

namespace rs
{
    //this class makes it easy to fork then exec a new process. It will
    //automatically send SIGTERM to the child when the class goes out of scope.
    class SpawnProcess
    {
    public:
        SpawnProcess();
        ~SpawnProcess();
        void Spawn(std::string cmd, std::string args="");

    private:
        pid_t m_pid;
        bool spawned;
    };

    //this class generates two serial ports that are connected to each other, designed for testing programs that talk over a serial port
    class SerialTB
    {
    public:
        SerialTB();
        ~SerialTB();

        //Start() takes in the port the UUT uses, and returns
        //the name of the testing port to use
        int Start(std::string &test_port, std::string &uut_port);

    private:
        SpawnProcess socat_proc;
        bool started;
    };


    void wait_for_param(const char *param_name, int timeout_seconds);
    void wait_for_subscriber(ros::Publisher pub, int timeout_seconds);
    void wait_for_publisher(ros::Subscriber pub, int timeout_seconds);


    //this class is meant to easy testing nodes, it acquires and holds onto received messages
    template <class mtype> class SubscriberTB
    {
    public:
        SubscriberTB()
        {}
        ~SubscriberTB()
        {}
        void Init(std::string topic)
        {
            ros::NodeHandle n;

            m_sub = n.subscribe(topic.c_str(), 1, &SubscriberTB<mtype>::Callback, this);

        }
        void Callback(const typename mtype::ConstPtr& msg)
        {
            m_msgs.push_back(*msg);
        }

        mtype GetLatestMsg()
        {
            return m_msgs.back();
        }

        mtype GetAllMsgs()
        {
            return m_msgs;
        }

        void ClearMsgs()
        {
            m_msgs.clear();
        }

    private:
        std::vector<mtype> m_msgs;
        ros::Subscriber m_sub;

    };
};

#endif
