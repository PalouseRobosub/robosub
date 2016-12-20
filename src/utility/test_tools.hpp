#ifndef TEST_TOOLS_HPP
#define TEST_TOOLS_HPP

#include "ros/ros.h"
#include <vector>
#include <string>

namespace rs
{
//this class makes it easy to fork then exec a new process. It will
//automatically send SIGTERM to the child when the class goes out of scope.
class SpawnProcess
{
public:
    SpawnProcess();
    ~SpawnProcess();
    void Spawn(std::string cmd, std::string args = "");

private:
    pid_t m_pid;
    bool spawned;
};

//this class generates two serial ports that are connected to each other,
//designed for testing programs that talk over a serial port
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


//this class is meant to easy testing nodes, it acquires and holds onto
//received messages
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

        m_sub = n.subscribe(topic.c_str(), 1,
                            &SubscriberTB<mtype>::Callback, this);
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

//this class is similar to the SubscriberTB, but is meant for analyzing a
//1-D stream of data. The format_callback should be a function that takes
//in the message datatype, and returns 1 double value, which is stored and
//later analyzed.
template <class mtype> class SubscriberAnalyzer
{
public:
    SubscriberAnalyzer()
    {}
    ~SubscriberAnalyzer()
    {}
    void Init(std::string topic,
              double (*format_callback)(const typename mtype::ConstPtr&))
    {
        ros::NodeHandle n;

        m_sub = n.subscribe(topic.c_str(), 1,
                            &SubscriberAnalyzer<mtype>::Callback, this);

        m_format_function = format_callback;

        m_enabled = false;
    }

    void Callback(const typename mtype::ConstPtr& msg)
    {
        if(m_enabled == true)
        {
            double data = m_format_function(msg);
            m_data.push_back(data);
        }
    }

    void Start()
    {
        m_enabled = true;
    }

    void Stop()
    {
        m_enabled = false;
    }

    void ClearData()
    {
        m_data.clear();
    }

    double GetMin()
    {
        if(m_data.size() == 0)
        {
            ROS_ERROR("no data collected, can't calculate min!");
            return 0.0;
        }

        double min = m_data[0];
        for (unsigned int i = 1; i < m_data.size(); ++i)
        {
            if (m_data[i] < min)
            {
                min = m_data[i];
            }
        }

        return min;
    }

    double GetMax()
    {
        if(m_data.size() == 0)
        {
            ROS_ERROR("no data collected, can't calculate max!");
            return 0.0;
        }

        double max = m_data[0];
        for (unsigned int i = 1; i < m_data.size(); ++i)
        {
            if (m_data[i] > max)
            {
                max = m_data[i];
            }
        }

        return max;
    }

    double GetAverage()
    {
        if(m_data.size() == 0)
        {
            ROS_ERROR("no data collected, can't calculate average!");
            return 0.0;
        }

        double sum = 0.0;
        for (unsigned int i = 0; i < m_data.size(); ++i)
        {
                sum += m_data[i];
        }

        return sum/m_data.size();
    }

    double GetStandardDeviation()
    {
        if(m_data.size() == 0)
        {
            ROS_ERROR("no data collected, "
                      "can't calculate standard deviation!");
            return 0.0;
        }

        double average = GetAverage();
        double sum = 0.0;
        for (unsigned int i = 0; i < m_data.size(); ++i)
        {
            sum += pow(m_data[i] - average, 2);
        }

        return sqrt(sum/m_data.size());
    }

private:
    std::vector<double> m_data;
    ros::Subscriber m_sub;
    double (*m_format_function)(const typename mtype::ConstPtr&);
    bool m_enabled;
};
};

#endif // TEST_TOOLS_HPP
