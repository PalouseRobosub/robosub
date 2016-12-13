#ifndef THROTTLEDPUBLISHER_HPP
#define THROTTLEDPUBLISHER_HPP

#include <ros/ros.h>
#include <string>

namespace rs
{
template <class msg> class ThrottledPublisher
{
    private:
        ros::Time nextPubTime;
        ros::Duration rate;
        ros::Publisher pub;
    public:
        ThrottledPublisher(std::string  topicName, int queueSize, float hz)
        {
            ros::NodeHandle nh;
            pub = nh.advertise<msg>(topicName, queueSize);
            this->rate = ros::Duration(1.0/hz);
            nextPubTime = ros::Time::now();
        }

        ~ThrottledPublisher()
        {}

        void publish(msg message)
        {
            if (nextPubTime <= ros::Time::now())
            {
                pub.publish(message);
                nextPubTime = ros::Time::now() + rate;
            }
        }
};
};

#endif // THROTTLEDPUBLISHER_HPP
