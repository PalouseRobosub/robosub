#ifndef THROTTLEDSUBSCRIBER_HPP
#define THROTTLEDSUBSCRIBER_HPP

#include <ros/ros.h>
#include <string>

namespace rs
{
template <class msg> class ThrottledSubscriber
{
    private:
        ros::Time nextSubTime;
        ros::Duration rate;
        ros::Subscriber sub;
        void (*callback)(const boost::shared_ptr<const msg>&);
        
        void internalCallback(const boost::shared_ptr<const msg>& message)
        {
            if (nextSubTime <= ros::Time::now())
            {
                ROS_INFO("Got into internal callback if clause");
                this->callback(message);
                nextSubTime = ros::Time::now() + rate;
                ROS_INFO_STREAM("rate: " << rate << " nextTime: " << nextSubTime);
            }
        }

    public:
        ThrottledSubscriber(const std::string topicName, uint32_t queueSize, void(*callback)(const boost::shared_ptr<const msg>&),
                            float hz)
        {
            ros::NodeHandle nh;
            this->callback = callback;
            sub = nh.subscribe(topicName, queueSize, &rs::ThrottledSubscriber<msg>::internalCallback, this);
            this->rate = ros::Duration(1.0/hz);
            nextSubTime = ros::Time::now();
        }

        ~ThrottledSubscriber()
        {}
};
};

#endif // THROTTLEDSUBSCRIBER_HPP
